#!/usr/bin/env python3
"""Check Betaflight SITL's parsed GPS data over MSP."""

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from msp_core.client import MspClient  # noqa: E402
from msp_core.protocol import MSP_RAW_GPS, MspProtocolError  # noqa: E402


GPS_FORMAT = "<BBiiHHHH"
GPS_SIZE = struct.calcsize(GPS_FORMAT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    args = parser.parse_args()

    try:
        with MspClient(args.host, args.port) as client:
            payload = client.request(MSP_RAW_GPS)
        if len(payload) != GPS_SIZE:
            raise ValueError(f"expected {GPS_SIZE} GPS bytes, received {len(payload)}")
        fix, satellites, latitude, longitude, altitude, speed, course, pdop = struct.unpack(GPS_FORMAT, payload)
        if not fix:
            raise ValueError("Betaflight reports no GPS fix")
        if not satellites:
            raise ValueError("Betaflight reports zero satellites")
        if abs(latitude) > 90 * 10**7 or abs(longitude) > 180 * 10**7:
            raise ValueError("Betaflight reports invalid coordinates")
    except (OSError, TimeoutError, ValueError, MspProtocolError) as exc:
        print(f"GPS check failed: {exc}", file=sys.stderr)
        return 1

    print(
        f"GPS OK: fix={fix} satellites={satellites} "
        f"latitude={latitude / 1e7:.7f} longitude={longitude / 1e7:.7f} "
        f"altitude={altitude}m speed={speed / 100:.2f}m/s "
        f"course={course / 10:.1f}deg pdop={pdop}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
