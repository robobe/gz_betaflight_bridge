#pragma once

#include <array>
#include <mutex>
#include <optional>
#include <string>

#include <gz/msgs/altimeter.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/transport/Node.hh>

#include "betaflight_gazebo_bridge/Config.hh"
#include "betaflight_gazebo_bridge/Packets.hh"

namespace betaflight_gazebo_bridge
{

struct SensorSnapshot
{
    double altitude{0.0};
    double verticalVelocity{0.0};
    std::array<double, 3> angularVelocity{0.0, 0.0, 0.0};
    std::array<double, 3> linearAcceleration{0.0, 0.0, -9.80665};
    std::array<double, 4> orientationQuat{1.0, 0.0, 0.0, 0.0};
    double longitudeDeg{0.0};
    double latitudeDeg{0.0};
    double gpsAltitude{0.0};
    double velocityEast{0.0};
    double velocityNorth{0.0};
    double velocityUp{0.0};
    bool hasNavSat{false};
};

class GazeboStateSubscriber
{
public:
    explicit GazeboStateSubscriber(const GazeboConfig &config);
    std::optional<SensorSnapshot> Snapshot() const;
    bool HasImu() const;
    bool HasAltimeter() const;
    bool HasNavSat() const;

private:
    void OnImu(const gz::msgs::IMU &msg);
    void OnAltimeter(const gz::msgs::Altimeter &msg);
    void OnNavSat(const gz::msgs::NavSat &msg);

    mutable std::mutex mutex_;
    gz::transport::Node node_;
    SensorSnapshot snapshot_;
    bool hasImu_{false};
    bool hasAltimeter_{false};
    bool navsatRequired_{false};
};

class ActuatorPublisher
{
public:
    explicit ActuatorPublisher(const GazeboConfig &config);
    void Publish(const std::array<double, 4> &velocities);

private:
    gz::transport::Node node_;
    gz::transport::Node::Publisher publisher_;
};

class FdmBuilder
{
public:
    explicit FdmBuilder(FdmConfig config);
    FdmPacket Build(const SensorSnapshot &snapshot, double timestampSeconds) const;

private:
    FdmConfig config_;
};

}  // namespace betaflight_gazebo_bridge
