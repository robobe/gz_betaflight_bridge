#include <cassert>
#include <cmath>
#include <stdexcept>

#include "betaflight_gazebo_bridge/Config.hh"
#include "betaflight_gazebo_bridge/GazeboTransport.hh"
#include "betaflight_gazebo_bridge/MotorMapper.hh"
#include "betaflight_gazebo_bridge/Packets.hh"

using namespace betaflight_gazebo_bridge;

int main()
{
    static_assert(sizeof(FdmPacket) == 144);
    static_assert(sizeof(ServoPacket) == 16);
    static_assert(sizeof(RcPacket) == 40);

    ConfigLoader::Validate(BridgeConfig{});

    BridgeConfig duplicateMap;
    duplicateMap.motors.map = {0, 0, 2, 3};
    bool threw = false;
    try {
        ConfigLoader::Validate(duplicateMap);
    } catch (const std::runtime_error &) {
        threw = true;
    }
    assert(threw);

    MotorMapper mapper({1, 0, 3, 2});
    const auto mapped = mapper.Apply({10.0, 20.0, 30.0, 40.0});
    assert(mapped[0] == 20.0);
    assert(mapped[1] == 10.0);
    assert(mapped[2] == 40.0);
    assert(mapped[3] == 30.0);

    MotorVelocityConverter converter(100.0, 900.0);
    assert(converter.Convert(-1.0) == 100.0);
    assert(converter.Convert(0.0) == 100.0);
    assert(converter.Convert(0.5) == 500.0);
    assert(converter.Convert(1.0) == 900.0);
    assert(converter.Convert(2.0) == 900.0);

    threw = false;
    try {
        (void)converter.Convert(std::nan(""));
    } catch (const std::runtime_error &) {
        threw = true;
    }
    assert(threw);

    FdmBuilder builder(FdmConfig{});
    SensorSnapshot snapshot;
    snapshot.altitude = 100.0;
    snapshot.verticalVelocity = 2.0;
    const auto packet = builder.Build(snapshot, 12.5);
    assert(packet.timestamp == 12.5);
    assert(packet.positionXyz[2] == 100.0);
    assert(packet.velocityXyz[2] == 2.0);
    assert(packet.pressure > 90000.0);
    assert(packet.pressure < 101325.0);

    return 0;
}
