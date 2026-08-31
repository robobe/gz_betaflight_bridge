#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>

#include "betaflight_gazebo_bridge/Config.hh"

namespace betaflight_gazebo_bridge
{

std::array<std::uint8_t, 9> EncodeTfmini(std::optional<double> metres);

class RangefinderManager
{
public:
    RangefinderManager(const std::vector<RangefinderConfig> &configs, const MavlinkConfig &mavlink,
                       std::chrono::steady_clock::time_point startTime);
    ~RangefinderManager();
    void Update();
    void LogStatus() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace betaflight_gazebo_bridge
