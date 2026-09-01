#pragma once

#include <array>
#include <chrono>

#include "betaflight_gazebo_bridge/Config.hh"
#include "betaflight_gazebo_bridge/GazeboTransport.hh"
#include "betaflight_gazebo_bridge/MotorMapper.hh"
#include "betaflight_gazebo_bridge/Odometry.hh"
#include "betaflight_gazebo_bridge/Rangefinder.hh"
#include "betaflight_gazebo_bridge/UdpSocket.hh"

namespace betaflight_gazebo_bridge
{

class BridgeApp
{
public:
    explicit BridgeApp(BridgeConfig config);
    int Run();

private:
    void ReceiveMotorPackets();
    void PublishMotorCommandIfNeeded();
    void SendFdmIfNeeded();
    void SendMavlinkHeartbeatIfNeeded();
    void LogStatusIfNeeded();
    std::array<double, 4> ZeroMotors() const;

    BridgeConfig config_;
    UdpSocket motorSocket_;
    UdpSocket fdmSocket_;
    UdpSocket mavlinkSocket_;
    GazeboStateSubscriber stateSubscriber_;
    ActuatorPublisher actuatorPublisher_;
    FdmBuilder fdmBuilder_;
    MotorMapper motorMapper_;
    MotorVelocityConverter velocityConverter_;

    std::array<double, 4> lastMotorCommand_{0.0, 0.0, 0.0, 0.0};
    std::chrono::steady_clock::time_point lastMotorPacketTime_{};
    std::chrono::steady_clock::time_point lastFdmSendTime_{};
    std::chrono::steady_clock::time_point lastStatusLogTime_{};
    std::chrono::steady_clock::time_point startTime_;
    RangefinderManager rangefinders_;
    OdometryManager odometry_;
    std::chrono::steady_clock::time_point lastMavlinkHeartbeatTime_{};
    bool hasMavlinkOutput_{};

    bool hasMotorPacket_{false};
    bool motorTimedOut_{false};
    bool loggedFirstMotor_{false};
    bool loggedFirstFdm_{false};
    std::uint64_t motorPackets_{0};
    std::uint64_t malformedMotorPackets_{0};
    std::uint64_t fdmPackets_{0};
};

}  // namespace betaflight_gazebo_bridge
