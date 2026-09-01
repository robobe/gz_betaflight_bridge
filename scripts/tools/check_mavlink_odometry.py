#!/usr/bin/env python3
"""Check the bridge's MAVLink heartbeat and ODOMETRY UDP output."""

import argparse
import math
import socket
import struct
import sys
import time


CRC_EXTRAS = {0: 50, 331: 91}


def crc_x25(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        byte ^= crc & 0xFF
        byte ^= (byte << 4) & 0xFF
        crc = ((crc >> 8) ^ (byte << 8) ^ (byte << 3) ^ (byte >> 4)) & 0xFFFF
    return crc


def parse_packet(packet: bytes) -> tuple[int, int, int, bytes]:
    if len(packet) < 12 or packet[0] != 0xFD:
        raise ValueError("not a MAVLink 2 packet")
    payload_size, incompat_flags = packet[1], packet[2]
    frame_size = 12 + payload_size + (13 if incompat_flags & 1 else 0)
    if len(packet) != frame_size:
        raise ValueError(f"MAVLink frame length is {len(packet)}, expected {frame_size}")
    message_id = int.from_bytes(packet[7:10], "little")
    payload = packet[10 : 10 + payload_size]
    if message_id in CRC_EXTRAS:
        expected = crc_x25(packet[1 : 10 + payload_size] + bytes([CRC_EXTRAS[message_id]]))
        actual, = struct.unpack_from("<H", packet, 10 + payload_size)
        if actual != expected:
            raise ValueError("bad MAVLink checksum")
    return message_id, packet[5], packet[6], payload


def describe_odometry(payload: bytes) -> str:
    if len(payload) < 230:
        raise ValueError("short ODOMETRY payload")
    payload = payload.ljust(233, b"\0")
    time_usec, *values = struct.unpack_from("<Q13f", payload)
    position, quaternion = values[:3], values[3:7]
    linear, angular = values[7:10], values[10:13]
    pose_covariance = struct.unpack_from("<21f", payload, 60)
    velocity_covariance = struct.unpack_from("<21f", payload, 144)
    frame_id, child_frame_id, reset_counter, estimator_type, quality = struct.unpack_from("<BBBBb", payload, 228)
    if (frame_id, child_frame_id) != (1, 12):
        raise ValueError(f"unexpected ODOMETRY frames {frame_id}/{child_frame_id}")
    if estimator_type != 1 or quality != 0:
        raise ValueError(f"unexpected estimator type or quality {estimator_type}/{quality}")
    if not math.isnan(pose_covariance[0]) or not math.isnan(velocity_covariance[0]):
        raise ValueError("ODOMETRY covariance is not marked unknown")
    if not all(math.isfinite(value) for value in position + quaternion + linear + angular):
        raise ValueError("ODOMETRY contains non-finite pose or velocity")
    if not math.isclose(math.sqrt(sum(value * value for value in quaternion)), 1.0, abs_tol=1e-4):
        raise ValueError("ODOMETRY quaternion is not normalized")
    return (f"time={time_usec}us position={tuple(round(v, 3) for v in position)} "
            f"q={tuple(round(v, 4) for v in quaternion)} velocity={tuple(round(v, 3) for v in linear)} "
            f"angular={tuple(round(v, 3) for v in angular)} reset={reset_counter}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=158)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    if not 1 <= args.system_id <= 255 or not 1 <= args.component_id <= 255:
        parser.error("system/component IDs must be 1..255")
    if args.timeout <= 0:
        parser.error("timeout must be positive")

    heartbeat = False
    odometry = None
    deadline = time.monotonic() + args.timeout
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((args.address, args.port))
            while time.monotonic() < deadline and (not heartbeat or odometry is None):
                sock.settimeout(max(0.01, deadline - time.monotonic()))
                message_id, system_id, component_id, payload = parse_packet(sock.recv(512))
                if (system_id, component_id) != (args.system_id, args.component_id):
                    continue
                if message_id == 0:
                    if len(payload) < 9 or payload[4] != 18 or payload[5] != 8:
                        raise ValueError("unexpected HEARTBEAT vehicle or autopilot type")
                    heartbeat = True
                elif message_id == 331:
                    odometry = describe_odometry(payload)
    except (OSError, ValueError) as exc:
        print(f"MAVLink odometry check failed: {exc}", file=sys.stderr)
        return 1

    if not heartbeat or odometry is None:
        missing = "heartbeat" if not heartbeat else "odometry"
        print(f"MAVLink odometry check failed: no {missing} within {args.timeout:g}s", file=sys.stderr)
        return 1
    print(f"MAVLink odometry OK: heartbeat system={args.system_id} component={args.component_id}; {odometry}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
