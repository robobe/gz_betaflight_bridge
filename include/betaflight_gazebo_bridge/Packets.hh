#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

namespace betaflight_gazebo_bridge
{

struct FdmPacket
{
    double timestamp;
    double imuAngularVelocityRpy[3];
    double imuLinearAccelerationXyz[3];
    double imuOrientationQuat[4];
    double velocityXyz[3];
    double positionXyz[3];
    double pressure;
};

struct ServoPacket
{
    float motorSpeed[4];
};

struct RcPacket
{
    double timestamp;
    std::uint16_t channels[16];
};

static_assert(std::is_standard_layout_v<FdmPacket>);
static_assert(std::is_trivially_copyable_v<FdmPacket>);
static_assert(sizeof(FdmPacket) == 18 * sizeof(double));

static_assert(std::is_standard_layout_v<ServoPacket>);
static_assert(std::is_trivially_copyable_v<ServoPacket>);
static_assert(sizeof(ServoPacket) == 4 * sizeof(float));

static_assert(std::is_standard_layout_v<RcPacket>);
static_assert(std::is_trivially_copyable_v<RcPacket>);
static_assert(sizeof(RcPacket) == sizeof(double) + 16 * sizeof(std::uint16_t));

}  // namespace betaflight_gazebo_bridge

