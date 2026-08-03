#include "betaflight_gazebo_bridge/MotorMapper.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace betaflight_gazebo_bridge
{

std::array<double, 4> SitlPacketToBetaflightMotorOrder(const std::array<double, 4> &packetMotors)
{
    // Betaflight SITL's Gazebo UDP output is deliberately serialized as
    // [M2, M3, M4, M1], not the flight controller's logical [M1, M2, M3, M4]
    // order. Normalize it here so motors.map consistently refers to logical
    // zero-based Betaflight motor indices.
    return {packetMotors[3], packetMotors[0], packetMotors[1], packetMotors[2]};
}

MotorMapper::MotorMapper(std::array<int, 4> map)
    : map_(map)
{
}

std::array<double, 4> MotorMapper::Apply(const std::array<double, 4> &betaflightMotors) const
{
    std::array<double, 4> gazeboMotors{};
    for (std::size_t gazeboIndex = 0; gazeboIndex < gazeboMotors.size(); ++gazeboIndex) {
        gazeboMotors[gazeboIndex] = betaflightMotors[static_cast<std::size_t>(map_[gazeboIndex])];
    }
    return gazeboMotors;
}

MotorVelocityConverter::MotorVelocityConverter(const double minVelocityRadS, const double maxVelocityRadS)
    : minVelocityRadS_(minVelocityRadS),
      maxVelocityRadS_(maxVelocityRadS)
{
    if (minVelocityRadS_ < 0.0 || maxVelocityRadS_ <= minVelocityRadS_) {
        throw std::runtime_error("Invalid motor velocity conversion range");
    }
}

double MotorVelocityConverter::Convert(const double normalizedCommand) const
{
    if (!std::isfinite(normalizedCommand)) {
        throw std::runtime_error("Motor command is not finite");
    }
    const double clamped = std::clamp(normalizedCommand, 0.0, 1.0);
    return minVelocityRadS_ + clamped * (maxVelocityRadS_ - minVelocityRadS_);
}

}  // namespace betaflight_gazebo_bridge
