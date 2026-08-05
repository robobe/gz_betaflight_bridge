#!/usr/bin/env python3
"""Receive Betaflight SITL motor packets from UDP 9002."""

import argparse
import socket
import struct


SERVO_PACKET_FORMAT = "<4f"
SERVO_PACKET_SIZE = struct.calcsize(SERVO_PACKET_FORMAT)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9002)
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((args.ip, args.port))
        print(f"Listening for {SERVO_PACKET_SIZE}-byte motor packets on {args.ip}:{args.port}")

        while True:
            data, address = sock.recvfrom(1024)
            if len(data) != SERVO_PACKET_SIZE:
                print(f"{address}: ignored {len(data)} bytes")
                continue

            motors = struct.unpack(SERVO_PACKET_FORMAT, data)
            print(f"{address}: {motors}")


if __name__ == "__main__":
    main()
