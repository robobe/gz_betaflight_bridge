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

}  // namespace

GazeboStateSubscriber::GazeboStateSubscriber(const GazeboConfig &config)
{
    if (!node_.Subscribe(config.imuTopic, &GazeboStateSubscriber::OnImu, this)) {
        throw std::runtime_error("Failed to subscribe to IMU topic: " + config.imuTopic);
    }
    if (!node_.Subscribe(config.altimeterTopic, &GazeboStateSubscriber::OnAltimeter, this)) {
        throw std::runtime_error("Failed to subscribe to altimeter topic: " + config.altimeterTopic);
    }
    spdlog::info("Subscribed to IMU [{}] and altimeter [{}]", config.imuTopic, config.altimeterTopic);
}

std::optional<SensorSnapshot> GazeboStateSubscriber::Snapshot() const
{
    std::lock_guard lock(mutex_);
    if (!hasImu_ || !hasAltimeter_) {
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
    for (std::size_t i = 0; i < 3; ++i) {
        packet.imuAngularVelocityRpy[i] = snapshot.angularVelocity[i];
        packet.imuLinearAccelerationXyz[i] = snapshot.linearAcceleration[i];
        packet.velocityXyz[i] = 0.0;
        packet.positionXyz[i] = 0.0;
    }
    packet.velocityXyz[2] = snapshot.verticalVelocity;
    packet.positionXyz[2] = snapshot.altitude;
    for (std::size_t i = 0; i < 4; ++i) {
        packet.imuOrientationQuat[i] = snapshot.orientationQuat[i];
    }
    packet.pressure = config_.pressureMode == "zero" ? 0.0 : PressureFromAltitude(snapshot.altitude, config_.seaLevelPressurePa);
    return packet;
}

}  // namespace betaflight_gazebo_bridge

