#include "betaflight_gazebo_bridge/GazeboTransport.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace betaflight_gazebo_bridge
{
namespace
{

double PressureFromAltitude(double altitudeMeters, const double seaLevelPressurePa)
{
    if (!std::isfinite(altitudeMeters)) {
        altitudeMeters = 0.0;
    }
    altitudeMeters = std::clamp(altitudeMeters, -1000.0, 44000.0);
    const double base = std::max(0.01, 1.0 - 2.25577e-5 * altitudeMeters);
    return seaLevelPressurePa * std::pow(base, 5.25588);
}

std::array<double, 3> FluVectorToFrdSensor(const std::array<double, 3> &flu)
{
    return {flu[0], -flu[1], -flu[2]};
}

std::array<double, 4> FluBodyToEnuWorldQuatToGazeboBridgeQuat(const std::array<double, 4> &quat)
{
    // Betaflight SITL_GAZEBO expects the packet quaternion as if the Gazebo IMU
    // sensor is Rx(pi) from the FLU body: q_plugin = Rx(pi) * q_flu_enu * Rx(pi).
    // q and -q are equivalent, so keep w positive for easier log inspection.
    return {quat[0], quat[1], -quat[2], -quat[3]};
}

}  // namespace

GazeboStateSubscriber::GazeboStateSubscriber(const GazeboConfig &config)
    : navsatRequired_(config.navsatTopic.has_value())
{
    if (!node_.Subscribe(config.imuTopic, &GazeboStateSubscriber::OnImu, this)) {
        throw std::runtime_error("Failed to subscribe to IMU topic: " + config.imuTopic);
    }
    if (!node_.Subscribe(config.altimeterTopic, &GazeboStateSubscriber::OnAltimeter, this)) {
        throw std::runtime_error("Failed to subscribe to altimeter topic: " + config.altimeterTopic);
    }
    if (config.navsatTopic) {
        if (!node_.Subscribe(*config.navsatTopic, &GazeboStateSubscriber::OnNavSat, this)) {
            throw std::runtime_error("Failed to subscribe to NavSat topic: " + *config.navsatTopic);
        }
        spdlog::info("Subscribed to IMU [{}], altimeter [{}], and NavSat [{}]",
                     config.imuTopic, config.altimeterTopic, *config.navsatTopic);
    } else {
        spdlog::info("Subscribed to IMU [{}] and altimeter [{}]", config.imuTopic, config.altimeterTopic);
    }
}

std::optional<SensorSnapshot> GazeboStateSubscriber::Snapshot() const
{
    std::lock_guard lock(mutex_);
    if (!hasImu_ || !hasAltimeter_ || (navsatRequired_ && !snapshot_.hasNavSat)) {
        return std::nullopt;
    }
    return snapshot_;
}

bool GazeboStateSubscriber::HasImu() const
{
    std::lock_guard lock(mutex_);
    return hasImu_;
}

bool GazeboStateSubscriber::HasAltimeter() const
{
    std::lock_guard lock(mutex_);
    return hasAltimeter_;
}

bool GazeboStateSubscriber::HasNavSat() const
{
    std::lock_guard lock(mutex_);
    return snapshot_.hasNavSat;
}

void GazeboStateSubscriber::OnImu(const gz::msgs::IMU &msg)
{
    std::lock_guard lock(mutex_);
    if (msg.has_angular_velocity()) {
        snapshot_.angularVelocity = {msg.angular_velocity().x(), msg.angular_velocity().y(), msg.angular_velocity().z()};
    }
    if (msg.has_linear_acceleration()) {
        snapshot_.linearAcceleration = {msg.linear_acceleration().x(), msg.linear_acceleration().y(), msg.linear_acceleration().z()};
    }
    if (msg.has_orientation()) {
        snapshot_.orientationQuat = {msg.orientation().w(), msg.orientation().x(), msg.orientation().y(), msg.orientation().z()};
    }
    hasImu_ = true;
}

void GazeboStateSubscriber::OnAltimeter(const gz::msgs::Altimeter &msg)
{
    std::lock_guard lock(mutex_);
    snapshot_.altitude = msg.vertical_position();
    snapshot_.verticalVelocity = msg.vertical_velocity();
    hasAltimeter_ = true;
}

void GazeboStateSubscriber::OnNavSat(const gz::msgs::NavSat &msg)
{
    std::lock_guard lock(mutex_);
    snapshot_.longitudeDeg = msg.longitude_deg();
    snapshot_.latitudeDeg = msg.latitude_deg();
    snapshot_.gpsAltitude = msg.altitude();
    snapshot_.velocityEast = msg.velocity_east();
    snapshot_.velocityNorth = msg.velocity_north();
    snapshot_.velocityUp = msg.velocity_up();
    snapshot_.hasNavSat = true;
}

ActuatorPublisher::ActuatorPublisher(const GazeboConfig &config)
    : publisher_(node_.Advertise<gz::msgs::Actuators>(config.actuatorTopic))
{
    if (!publisher_) {
        throw std::runtime_error("Failed to advertise actuator topic: " + config.actuatorTopic);
    }
    spdlog::info("Publishing actuators on [{}]", config.actuatorTopic);
}

void ActuatorPublisher::Publish(const std::array<double, 4> &velocities)
{
    gz::msgs::Actuators msg;
    for (const double velocity : velocities) {
        msg.add_velocity(velocity);
    }
    publisher_.Publish(msg);
}

FdmBuilder::FdmBuilder(FdmConfig config)
    : config_(std::move(config))
{
}

FdmPacket FdmBuilder::Build(const SensorSnapshot &snapshot, const double timestampSeconds) const
{
    FdmPacket packet{};
    packet.timestamp = timestampSeconds;
    const bool gazeboBridgeFrame = config_.frameMode == "gazebo_bridge";
    const auto angularVelocity = gazeboBridgeFrame ? FluVectorToFrdSensor(snapshot.angularVelocity) : snapshot.angularVelocity;
    const auto linearAcceleration = gazeboBridgeFrame ? FluVectorToFrdSensor(snapshot.linearAcceleration) : snapshot.linearAcceleration;
    const auto orientationQuat = gazeboBridgeFrame ? FluBodyToEnuWorldQuatToGazeboBridgeQuat(snapshot.orientationQuat) : snapshot.orientationQuat;

    for (std::size_t i = 0; i < 3; ++i) {
        packet.imuAngularVelocityRpy[i] = angularVelocity[i];
        packet.imuLinearAccelerationXyz[i] = linearAcceleration[i];
        packet.velocityXyz[i] = 0.0;
        packet.positionXyz[i] = 0.0;
    }
    if (snapshot.hasNavSat) {
        packet.positionXyz[0] = snapshot.longitudeDeg;
        packet.positionXyz[1] = snapshot.latitudeDeg;
        packet.velocityXyz[0] = snapshot.velocityEast;
        packet.velocityXyz[1] = snapshot.velocityNorth;
    }
    const bool gpsAltitude = config_.altitudeSource == "gps" && snapshot.hasNavSat;
    packet.positionXyz[2] = gpsAltitude ? snapshot.gpsAltitude : snapshot.altitude;
    packet.velocityXyz[2] = gpsAltitude ? snapshot.velocityUp : snapshot.verticalVelocity;
    for (std::size_t i = 0; i < 4; ++i) {
        packet.imuOrientationQuat[i] = orientationQuat[i];
    }
    packet.pressure = config_.pressureMode == "zero" ? 0.0 : PressureFromAltitude(snapshot.altitude, config_.seaLevelPressurePa);
    return packet;
}

}  // namespace betaflight_gazebo_bridge
