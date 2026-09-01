#include "betaflight_gazebo_bridge/Config.hh"

#include <algorithm>
#include <set>
#include <stdexcept>

#include <arpa/inet.h>

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
    if (gazebo && gazebo["navsat_topic"]) {
        config.gazebo.navsatTopic = gazebo["navsat_topic"].as<std::string>();
    }
    ReadScalar(gazebo, "actuator_topic", config.gazebo.actuatorTopic);

    const auto fdm = root["fdm"];
    ReadScalar(fdm, "rate_hz", config.fdm.rateHz);
    ReadScalar(fdm, "frame_mode", config.fdm.frameMode);
    ReadScalar(fdm, "pressure_mode", config.fdm.pressureMode);
    ReadScalar(fdm, "altitude_source", config.fdm.altitudeSource);
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

    const auto mavlink = root["mavlink"];
    ReadScalar(mavlink, "address", config.mavlink.address);
    config.mavlink.port = ReadPort(mavlink, "port", config.mavlink.port);
    int systemId = config.mavlink.systemId;
    int componentId = config.mavlink.componentId;
    ReadScalar(mavlink, "system_id", systemId);
    ReadScalar(mavlink, "component_id", componentId);
    if (systemId < 1 || systemId > 255 || componentId < 1 || componentId > 255) {
        throw std::runtime_error("mavlink source IDs must be in range 1..255");
    }
    config.mavlink.systemId = static_cast<std::uint8_t>(systemId);
    config.mavlink.componentId = static_cast<std::uint8_t>(componentId);

    const auto odometry = root["odometry"];
    ReadScalar(odometry, "enable", config.odometry.enabled);
    ReadScalar(odometry, "gazebo_topic", config.odometry.gazeboTopic);

    const auto rangefinders = root["rangefinders"];
    if (rangefinders && !rangefinders.IsSequence()) {
        throw std::runtime_error("rangefinders must be a list");
    }
    for (const auto &node : rangefinders) {
        RangefinderConfig rangefinder;
        ReadScalar(node, "enable", rangefinder.enabled);
        ReadScalar(node, "name", rangefinder.name);
        ReadScalar(node, "gazebo_topic", rangefinder.gazeboTopic);
        ReadScalar(node, "output", rangefinder.output);
        ReadScalar(node, "sitl_address", rangefinder.sitlAddress);
        rangefinder.sitlPort = ReadPort(node, "sitl_port", 0);
        ReadScalar(node, "mavlink_message", rangefinder.mavlinkMessage);
        ReadScalar(node, "orientation", rangefinder.orientation);
        if (node["sensor_id"]) {
            const int id = node["sensor_id"].as<int>();
            if (id < 0 || id > 255) throw std::runtime_error("rangefinder sensor_id must be in range 0..255");
            rangefinder.sensorId = static_cast<std::uint8_t>(id);
        }
        config.rangefinders.push_back(std::move(rangefinder));
    }

    Validate(config);
    return config;
}

void ConfigLoader::Validate(const BridgeConfig &config)
{
    in_addr mavlinkAddress{};
    if (inet_pton(AF_INET, config.mavlink.address.c_str(), &mavlinkAddress) != 1) {
        throw std::runtime_error("mavlink.address must be an IPv4 address");
    }
    if (config.gazebo.imuTopic.empty() || config.gazebo.altimeterTopic.empty() || config.gazebo.actuatorTopic.empty()) {
        throw std::runtime_error("Gazebo topics must not be empty");
    }
    if (config.gazebo.navsatTopic && config.gazebo.navsatTopic->empty()) {
        throw std::runtime_error("gazebo.navsat_topic must not be empty");
    }
    if (config.odometry.enabled && config.odometry.gazeboTopic.empty()) {
        throw std::runtime_error("odometry.gazebo_topic must not be empty");
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
    if (config.fdm.altitudeSource != "altimeter" && config.fdm.altitudeSource != "gps") {
        throw std::runtime_error("fdm.altitude_source must be 'altimeter' or 'gps'");
    }
    if (config.fdm.altitudeSource == "gps" && !config.gazebo.navsatTopic) {
        throw std::runtime_error("fdm.altitude_source 'gps' requires gazebo.navsat_topic");
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

    std::set<std::string> names, topics, endpoints;
    std::set<std::uint8_t> sensorIds;
    bool hasObstacleDistance = false;
    for (const auto &rangefinder : config.rangefinders) {
        if (rangefinder.name.empty() || !names.insert(rangefinder.name).second) {
            throw std::runtime_error("rangefinder names must be non-empty and unique");
        }
        if (rangefinder.gazeboTopic.empty() || !topics.insert(rangefinder.gazeboTopic).second) {
            throw std::runtime_error("rangefinder Gazebo topics must be non-empty and unique");
        }
        if (rangefinder.output == "tfmini") {
            if (rangefinder.sitlAddress.empty() || rangefinder.sitlPort == 0) {
                throw std::runtime_error("TFmini rangefinder requires sitl_address and sitl_port");
            }
            if (!endpoints.insert(rangefinder.sitlAddress + ":" + std::to_string(rangefinder.sitlPort)).second) {
                throw std::runtime_error("TFmini endpoints must be unique");
            }
        } else if (rangefinder.output == "mavlink") {
            if (rangefinder.orientation != "forward") {
                throw std::runtime_error("MAVLink rangefinder orientation must be 'forward'");
            }
            if (rangefinder.mavlinkMessage == "distance_sensor") {
                if (!rangefinder.sensorId || !sensorIds.insert(*rangefinder.sensorId).second) {
                    throw std::runtime_error("DISTANCE_SENSOR requires a unique sensor_id");
                }
            } else if (rangefinder.mavlinkMessage == "obstacle_distance") {
                if (hasObstacleDistance) throw std::runtime_error("only one OBSTACLE_DISTANCE rangefinder is supported");
                hasObstacleDistance = true;
            } else {
                throw std::runtime_error("mavlink_message must be 'distance_sensor' or 'obstacle_distance'");
            }
        } else {
            throw std::runtime_error("rangefinder output must be 'tfmini' or 'mavlink'");
        }
    }
}

}  // namespace betaflight_gazebo_bridge
