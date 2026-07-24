#pragma once

#include <array>

namespace betaflight_gazebo_bridge
{

class MotorMapper
{
public:
    explicit MotorMapper(std::array<int, 4> map);
    std::array<double, 4> Apply(const std::array<double, 4> &betaflightMotors) const;

private:
    std::array<int, 4> map_;
};

class MotorVelocityConverter
{
public:
    MotorVelocityConverter(double minVelocityRadS, double maxVelocityRadS);
    double Convert(double normalizedCommand) const;

private:
    double minVelocityRadS_{0.0};
    double maxVelocityRadS_{800.0};
};

}  // namespace betaflight_gazebo_bridge

