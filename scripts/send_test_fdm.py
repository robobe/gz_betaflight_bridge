#!/usr/bin/env python3
"""Send minimal Betaflight SITL FDM packets to UDP 9003."""

import argparse
import math
import socket
import struct
import time


FDM_PACKET_FORMAT = "<18d"
GRAVITY_MPS2 = 9.80665


def make_packet(timestamp: float) -> bytes:
    imu_angular_velocity_rpy = (0.0, 0.0, 0.0)
    imu_linear_acceleration_xyz = (0.0, 0.0, -GRAVITY_MPS2)
    imu_orientation_quat = (1.0, 0.0, 0.0, 0.0)
    velocity_xyz = (0.0, 0.0, 0.0)
    position_xyz = (0.0, 0.0, 0.0)
    pressure = 101325.0

    return struct.pack(
        FDM_PACKET_FORMAT,
        timestamp,
        *imu_angular_velocity_rpy,
        *imu_linear_acceleration_xyz,
        *imu_orientation_quat,
        *velocity_xyz,
        *position_xyz,
        pressure,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9003)
    parser.add_argument("--rate", type=float, default=500.0)
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    period = 1.0 / args.rate
    end_time = time.monotonic() + args.duration
    start_time = time.monotonic()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while time.monotonic() < end_time:
            timestamp = time.monotonic() - start_time
            sock.sendto(make_packet(timestamp), (args.ip, args.port))
            time.sleep(period)


if __name__ == "__main__":
    main()

