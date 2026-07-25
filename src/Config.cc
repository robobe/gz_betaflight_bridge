#include "betaflight_gazebo_bridge/Config.hh"

#include <algorithm>
#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace betaflight_gazebo_bridge
{
namespace
{

template <typename T>
void ReadScalar(const YAML::Node &node, const char *key, T &value)
{
    if (node && node[key]) {
        value = node[key].as<T>();
    }
}

std::uint16_t ReadPort(const YAML::Node &node, const char *key, std::uint16_t fallback)
{
    if (!node || !node[key]) {
        return fallback;
    }
    const int port = node[key].as<int>();
    if (port <= 0 || port > 65535) {
        throw std::runtime_error(std::string("Invalid UDP port for ") + key);
    }
    return static_cast<std::uint16_t>(port);
}

}  // namespace

BridgeConfig ConfigLoader::Load(const std::filesystem::path &path)
{
    BridgeConfig config;
    const YAML::Node root = YAML::LoadFile(path.string());

    const auto sitl = root["sitl"];
    ReadScalar(sitl, "address", config.sitl.address);
    config.sitl.motorPort = ReadPort(sitl, "motor_port", config.sitl.motorPort);
    config.sitl.fdmPort = ReadPort(sitl, "fdm_port", config.sitl.fdmPort);
    config.sitl.rcPort = ReadPort(sitl, "rc_port", config.sitl.rcPort);

    const auto gazebo = root["gazebo"];
    ReadScalar(gazebo, "imu_topic", config.gazebo.imuTopic);
    ReadScalar(gazebo, "altimeter_topic", config.gazebo.altimeterTopic);
    ReadScalar(gazebo, "actuator_topic", config.gazebo.actuatorTopic);

    const auto fdm = root["fdm"];
    ReadScalar(fdm, "rate_hz", config.fdm.rateHz);
    ReadScalar(fdm, "frame_mode", config.fdm.frameMode);
    ReadScalar(fdm, "pressure_mode", config.fdm.pressureMode);
    ReadScalar(fdm, "sea_level_pressure_pa", config.fdm.seaLevelPressurePa);

    const auto motors = root["motors"];
    if (motors && motors["map"]) {
        const auto map = motors["map"];
        if (!map.IsSequence() || map.size() != config.motors.map.size()) {
            throw std::runtime_error("motors.map must contain exactly four indices");
        }
        for (std::size_t i = 0; i < config.motors.map.size(); ++i) {
            config.motors.map[i] = map[i].as<int>();
        }
    }
    ReadScalar(motors, "min_rotor_velocity_rad_s", config.motors.minRotorVelocityRadS);
    ReadScalar(motors, "max_rotor_velocity_rad_s", config.motors.maxRotorVelocityRadS);
    ReadScalar(motors, "timeout_seconds", config.motors.timeoutSeconds);
    ReadScalar(motors, "publish_zero_on_timeout", config.motors.publishZeroOnTimeout);

    const auto logging = root["logging"];
    ReadScalar(logging, "level", config.logging.level);
    ReadScalar(logging, "status_period_seconds", config.logging.statusPeriodSeconds);
    ReadScalar(logging, "log_first_packets", config.logging.logFirstPackets);

    Validate(config);
    return config;
}

void ConfigLoader::Validate(const BridgeConfig &config)
{
    if (config.gazebo.imuTopic.empty() || config.gazebo.altimeterTopic.empty() || config.gazebo.actuatorTopic.empty()) {
        throw std::runtime_error("Gazebo topics must not be empty");
    }
    if (config.fdm.rateHz <= 0.0) {
        throw std::runtime_error("fdm.rate_hz must be positive");
    }
    if (config.fdm.frameMode != "gazebo_bridge" && config.fdm.frameMode != "passthrough") {
        throw std::runtime_error("fdm.frame_mode must be 'gazebo_bridge' or 'passthrough'");
    }
    if (config.fdm.seaLevelPressurePa <= 0.0) {
        throw std::runtime_error("fdm.sea_level_pressure_pa must be positive");
    }
    if (config.fdm.pressureMode != "from_altitude" && config.fdm.pressureMode != "zero") {
        throw std::runtime_error("fdm.pressure_mode must be 'from_altitude' or 'zero'");
    }
    if (config.motors.timeoutSeconds <= 0.0) {
        throw std::runtime_error("motors.timeout_seconds must be positive");
    }
    if (config.motors.minRotorVelocityRadS < 0.0 ||
        config.motors.maxRotorVelocityRadS <= config.motors.minRotorVelocityRadS) {
        throw std::runtime_error("motor velocity range is invalid");
    }

    std::set<int> seen;
    for (const int index : config.motors.map) {
        if (index < 0 || index > 3) {
            throw std::runtime_error("motors.map values must be in range 0..3");
        }
        if (!seen.insert(index).second) {
            throw std::runtime_error("motors.map must not contain duplicate indices");
        }
    }
}

}  // namespace betaflight_gazebo_bridge
