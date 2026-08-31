#!/usr/bin/env python3
"""Print Betaflight SITL build and runtime configuration over MSP."""

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from msp_core.client import MspClient  # noqa: E402
from msp_core.protocol import MspProtocolError  # noqa: E402


FEATURES = {
    0: "RX_PPM", 1: "RX_UDP", 2: "INFLIGHT_ACC_CAL", 3: "RX_SERIAL",
    4: "MOTOR_STOP", 5: "SERVO_TILT", 6: "SOFTSERIAL", 7: "GPS",
    8: "OPTICALFLOW", 9: "RANGEFINDER", 10: "TELEMETRY", 12: "3D",
    13: "RX_PARALLEL_PWM", 14: "RX_MSP", 15: "RSSI_ADC", 16: "LED_STRIP",
    17: "DASHBOARD", 18: "OSD", 20: "CHANNEL_FORWARDING", 21: "TRANSPONDER",
    22: "AIRMODE", 25: "RX_SPI", 27: "ESC_SENSOR", 28: "ANTI_GRAVITY",
}
SENSORS = ("ACC", "BARO", "MAG", "GPS", "RANGEFINDER", "GYRO", "OPTICALFLOW")
GPS_PROVIDERS = ("NMEA", "UBLOX", "MSP", "VIRTUAL", "DRONECAN")
SERIAL_FUNCTIONS = {0: "MSP", 1: "GPS", 6: "RX_SERIAL", 7: "BLACKBOX", 9: "MAVLINK", 15: "LIDAR_TF"}


def pstring(data: bytes, offset: int) -> tuple[str, int]:
    size = data[offset]
    return data[offset + 1 : offset + 1 + size].decode("ascii", "replace"), offset + 1 + size


def names(mask: int, choices: dict[int, str]) -> str:
    return ", ".join(name for bit, name in choices.items() if mask & (1 << bit)) or "none"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    args = parser.parse_args()

    try:
        with MspClient(args.host, args.port) as client:
            api = client.request(1)
            variant = client.request(2).decode("ascii", "replace")
            version_data = client.request(3)
            board_data = client.request(4)
            build = client.request(5)
            feature_mask, = struct.unpack("<I", client.request(36))
            status = client.request(101)
            gps = client.request(132)
            serial = client.request(54)

        version, _ = pstring(version_data, 3)
        target, _ = pstring(board_data, 8)
        sensor_mask, = struct.unpack_from("<H", status, 4)
        gps_provider = GPS_PROVIDERS[gps[0]] if gps[0] < len(GPS_PROVIDERS) else f"unknown ({gps[0]})"

        print(f"Betaflight: {variant} {version} ({target or board_data[:4].decode('ascii', 'replace')})")
        print(f"MSP API: {api[1]}.{api[2]} (protocol {api[0]})")
        print(f"Build: {build[:11].decode()} {build[11:19].decode()} git {build[19:26].decode()}")
        print(f"Features: {names(feature_mask, FEATURES)}")
        print(f"Sensors: {names(sensor_mask, dict(enumerate(SENSORS)))}")
        print(f"GPS: provider={gps_provider} auto_config={bool(gps[2])} auto_baud={bool(gps[3])}")
        print("Serial ports:")
        for offset in range(0, len(serial), 7):
            identifier, function_mask = struct.unpack_from("<BH", serial, offset)
            print(f"  {identifier}: {names(function_mask, SERIAL_FUNCTIONS)} (0x{function_mask:04x})")
    except (IndexError, OSError, struct.error, TimeoutError, UnicodeError, MspProtocolError) as exc:
        print(f"SITL inspection failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
