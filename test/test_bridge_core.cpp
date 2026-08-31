#include <cassert>
#include <cmath>
#include <stdexcept>

#include "betaflight_gazebo_bridge/Config.hh"
#include "betaflight_gazebo_bridge/GazeboTransport.hh"
#include "betaflight_gazebo_bridge/MotorMapper.hh"
#include "betaflight_gazebo_bridge/Packets.hh"
#include "betaflight_gazebo_bridge/Rangefinder.hh"

using namespace betaflight_gazebo_bridge;

int main()
{
    static_assert(sizeof(FdmPacket) == 144);
    static_assert(sizeof(ServoPacket) == 16);
    static_assert(sizeof(RcPacket) == 40);

    ConfigLoader::Validate(BridgeConfig{});
    assert(RangefinderConfig{}.enabled);

    const auto tfmini = EncodeTfmini(1.23);
    assert(tfmini[0] == 0x59 && tfmini[1] == 0x59);
    assert(tfmini[2] == 123 && tfmini[3] == 0);
    unsigned checksum = 0;
    for (std::size_t i = 0; i < 8; ++i) checksum += tfmini[i];
    assert(tfmini[8] == (checksum & 0xff));
    const auto invalidTfmini = EncodeTfmini(std::nan(""));
    assert(invalidTfmini[2] == (1201 & 0xff) && invalidTfmini[3] == (1201 >> 8));

    BridgeConfig duplicateMap;
    duplicateMap.motors.map = {0, 0, 2, 3};
    bool threw = false;
    try {
        ConfigLoader::Validate(duplicateMap);
    } catch (const std::runtime_error &) {
        threw = true;
    }
    assert(threw);

    BridgeConfig emptyNavSat;
    emptyNavSat.gazebo.navsatTopic = "";
    threw = false;
    try {
        ConfigLoader::Validate(emptyNavSat);
    } catch (const std::runtime_error &) {
        threw = true;
    }
    assert(threw);

    BridgeConfig gpsWithoutTopic;
    gpsWithoutTopic.fdm.altitudeSource = "gps";
    threw = false;
    try {
        ConfigLoader::Validate(gpsWithoutTopic);
    } catch (const std::runtime_error &) {
        threw = true;
    }
    assert(threw);

    const auto logicalMotors = SitlPacketToBetaflightMotorOrder({20.0, 30.0, 40.0, 10.0});
    assert(logicalMotors[0] == 10.0);  // M1
    assert(logicalMotors[1] == 20.0);  // M2
    assert(logicalMotors[2] == 30.0);  // M3
    assert(logicalMotors[3] == 40.0);  // M4

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
    snapshot.angularVelocity = {1.0, 2.0, 3.0};
    snapshot.linearAcceleration = {4.0, 5.0, 6.0};
    snapshot.orientationQuat = {0.5, 0.5, 0.5, 0.5};
    const auto packet = builder.Build(snapshot, 12.5);
    assert(packet.timestamp == 12.5);
    assert(packet.positionXyz[2] == 100.0);
    assert(packet.velocityXyz[2] == 2.0);
    assert(packet.imuAngularVelocityRpy[0] == 1.0);
    assert(packet.imuAngularVelocityRpy[1] == -2.0);
    assert(packet.imuAngularVelocityRpy[2] == -3.0);
    assert(packet.imuLinearAccelerationXyz[0] == 4.0);
    assert(packet.imuLinearAccelerationXyz[1] == -5.0);
    assert(packet.imuLinearAccelerationXyz[2] == -6.0);
    assert(packet.imuOrientationQuat[0] == 0.5);
    assert(packet.imuOrientationQuat[1] == 0.5);
    assert(packet.imuOrientationQuat[2] == -0.5);
    assert(packet.imuOrientationQuat[3] == -0.5);
    assert(packet.pressure > 90000.0);
    assert(packet.pressure < 101325.0);

    snapshot.longitudeDeg = 34.7818;
    snapshot.latitudeDeg = 32.0853;
    snapshot.gpsAltitude = 120.0;
    snapshot.velocityEast = 3.0;
    snapshot.velocityNorth = 4.0;
    snapshot.velocityUp = 5.0;
    snapshot.hasNavSat = true;
    const auto altimeterGpsPacket = builder.Build(snapshot, 12.5);
    assert(altimeterGpsPacket.positionXyz[0] == 34.7818);
    assert(altimeterGpsPacket.positionXyz[1] == 32.0853);
    assert(altimeterGpsPacket.positionXyz[2] == 100.0);
    assert(altimeterGpsPacket.velocityXyz[0] == 3.0);
    assert(altimeterGpsPacket.velocityXyz[1] == 4.0);
    assert(altimeterGpsPacket.velocityXyz[2] == 2.0);

    FdmConfig gpsAltitudeConfig;
    gpsAltitudeConfig.altitudeSource = "gps";
    const auto gpsPacket = FdmBuilder(gpsAltitudeConfig).Build(snapshot, 12.5);
    assert(gpsPacket.positionXyz[2] == 120.0);
    assert(gpsPacket.velocityXyz[2] == 5.0);

    FdmConfig passthroughConfig;
    passthroughConfig.frameMode = "passthrough";
    FdmBuilder passthroughBuilder(passthroughConfig);
    const auto passthroughPacket = passthroughBuilder.Build(snapshot, 12.5);
    assert(passthroughPacket.imuAngularVelocityRpy[1] == 2.0);
    assert(passthroughPacket.imuAngularVelocityRpy[2] == 3.0);
    assert(passthroughPacket.imuLinearAccelerationXyz[1] == 5.0);
    assert(passthroughPacket.imuLinearAccelerationXyz[2] == 6.0);
    assert(passthroughPacket.imuOrientationQuat[2] == 0.5);
    assert(passthroughPacket.imuOrientationQuat[3] == 0.5);

    return 0;
}
