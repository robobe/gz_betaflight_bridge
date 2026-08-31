#!/usr/bin/env python3
"""Check Betaflight's configured, detected, and measured rangefinder over MSP."""

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from msp_core.client import MspClient  # noqa: E402
from msp_core.protocol import (  # noqa: E402
    MSP_SENSOR_CONFIG, MSP_SONAR_ALTITUDE, MSP_STATUS, MspProtocolError,
)

RANGEFINDER_HARDWARE = ("NONE", "HCSR04", "TFMINI", "TF02", "MTF01", "MTF02")


def decode_responses(config: bytes, status: bytes, altitude: bytes) -> tuple[int, bool, int]:
    if len(config) < 4 or len(status) < 6 or len(altitude) != 4:
        raise ValueError("short MSP rangefinder response")
    return config[3], bool(struct.unpack_from("<H", status, 4)[0] & (1 << 4)), struct.unpack("<i", altitude)[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    args = parser.parse_args()
    try:
        with MspClient(args.host, args.port) as client:
            config = client.request(MSP_SENSOR_CONFIG)
            status = client.request(MSP_STATUS)
            altitude = client.request(MSP_SONAR_ALTITUDE)
        hardware, detected, centimetres = decode_responses(config, status, altitude)
    except (OSError, TimeoutError, ValueError, struct.error, MspProtocolError) as exc:
        print(f"Rangefinder check failed: {exc}", file=sys.stderr)
        return 1

    hardware_name = RANGEFINDER_HARDWARE[hardware] if hardware < len(RANGEFINDER_HARDWARE) else str(hardware)
    if not hardware or not detected or centimetres < 0:
        print(f"Rangefinder unavailable: hardware={hardware_name} detected={detected} altitude={centimetres}cm")
        return 1
    print(f"Rangefinder OK: hardware={hardware_name} detected=yes altitude={centimetres}cm ({centimetres / 100:.2f}m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
