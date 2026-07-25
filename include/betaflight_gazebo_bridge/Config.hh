#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>

namespace betaflight_gazebo_bridge
{

struct SitlConfig
{
    std::string address{"127.0.0.1"};
    std::uint16_t motorPort{9002};
    std::uint16_t fdmPort{9003};
    std::uint16_t rcPort{9004};
};

struct GazeboConfig
{
    std::string imuTopic{"/imu"};
    std::string altimeterTopic{"/altimeter"};
    std::string actuatorTopic{"/X3/gazebo/command/motor_speed"};
};

struct FdmConfig
{
    double rateHz{500.0};
    std::string frameMode{"gazebo_bridge"};
    std::string pressureMode{"from_altitude"};
    double seaLevelPressurePa{101325.0};
};

struct MotorConfig
{
    std::array<int, 4> map{0, 1, 2, 3};
    double minRotorVelocityRadS{0.0};
    double maxRotorVelocityRadS{800.0};
    double timeoutSeconds{0.10};
    bool publishZeroOnTimeout{true};
};

struct LoggingConfig
{
    std::string level{"info"};
    double statusPeriodSeconds{1.0};
    bool logFirstPackets{true};
};

struct BridgeConfig
{
    SitlConfig sitl;
    GazeboConfig gazebo;
    FdmConfig fdm;
    MotorConfig motors;
    LoggingConfig logging;
};

class ConfigLoader
{
public:
    static BridgeConfig Load(const std::filesystem::path &path);
    static void Validate(const BridgeConfig &config);
};

}  // namespace betaflight_gazebo_bridge
