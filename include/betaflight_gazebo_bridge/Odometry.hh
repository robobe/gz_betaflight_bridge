#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>

#include "betaflight_gazebo_bridge/Config.hh"

namespace betaflight_gazebo_bridge
{

struct OdometrySample
{
    std::uint64_t timeUsec{};
    std::array<double, 3> position{};
    std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> linearVelocity{};
    std::array<double, 3> angularVelocity{};
};

struct ConvertedOdometry
{
    std::uint64_t timeUsec{};
    std::array<float, 3> position{};
    std::array<float, 4> orientation{};
    std::array<float, 3> linearVelocity{};
    std::array<float, 3> angularVelocity{};
    std::uint8_t resetCounter{};
};

class OdometryConverter
{
public:
    std::optional<ConvertedOdometry> Convert(const OdometrySample &sample);

private:
    std::optional<std::array<double, 3>> origin_;
    std::optional<std::uint64_t> lastTimeUsec_;
    std::uint8_t resetCounter_{};
};

class OdometryManager
{
public:
    OdometryManager(const OdometryConfig &config, const MavlinkConfig &mavlink);
    ~OdometryManager();
    void Update();
    void LogStatus() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace betaflight_gazebo_bridge
