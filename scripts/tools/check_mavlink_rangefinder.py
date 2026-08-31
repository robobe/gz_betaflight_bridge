#!/usr/bin/env python3
"""Check the bridge's MAVLink heartbeat and rangefinder UDP output."""

import argparse
import socket
import struct
import sys
import time


MESSAGES = {0: ("HEARTBEAT", 50), 132: ("DISTANCE_SENSOR", 85), 330: ("OBSTACLE_DISTANCE", 23)}


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
    if message_id not in MESSAGES:
        return message_id, packet[5], packet[6], packet[10 : 10 + payload_size]
    expected = crc_x25(packet[1 : 10 + payload_size] + bytes([MESSAGES[message_id][1]]))
    actual, = struct.unpack_from("<H", packet, 10 + payload_size)
    if actual != expected:
        raise ValueError(f"bad {MESSAGES[message_id][0]} checksum")
    return message_id, packet[5], packet[6], packet[10 : 10 + payload_size]


def describe_range(message_id: int, payload: bytes, sensor_id: int) -> str:
    if message_id == 132:
        if len(payload) < 14:
            raise ValueError("short DISTANCE_SENSOR payload")
        minimum, maximum, current = struct.unpack_from("<HHH", payload, 4)
        sensor_type, actual_id, orientation, covariance = payload[10:14]
        if sensor_type != 0 or actual_id != sensor_id or orientation != 0 or covariance != 0xFF:
            raise ValueError("unexpected DISTANCE_SENSOR type, ID, orientation, or covariance")
        if not minimum <= current <= maximum:
            raise ValueError(f"DISTANCE_SENSOR reading {current}cm outside {minimum}..{maximum}cm")
        return f"DISTANCE_SENSOR id={actual_id} distance={current}cm ({current / 100:.2f}m) range={minimum}..{maximum}cm"

    if len(payload) < 167:
        raise ValueError("short OBSTACLE_DISTANCE payload")
    distances = struct.unpack_from("<72H", payload, 8)
    minimum, maximum = struct.unpack_from("<HH", payload, 152)
    if payload[156] != 0 or payload[157] != 0 or struct.unpack_from("<f", payload, 158)[0] != 0.0:
        raise ValueError("unexpected OBSTACLE_DISTANCE sensor type or increment")
    if struct.unpack_from("<f", payload, 162)[0] != 0.0 or payload[166] != 12:
        raise ValueError("OBSTACLE_DISTANCE is not forward-facing in MAV_FRAME_BODY_FRD")
    if any(distance != 0xFFFF for distance in distances[1:]):
        raise ValueError("OBSTACLE_DISTANCE has populated non-forward bins")
    reading = "unknown" if distances[0] == 0xFFFF else "no return" if distances[0] == maximum + 1 else f"{distances[0]}cm ({distances[0] / 100:.2f}m)"
    return f"OBSTACLE_DISTANCE forward={reading} range={minimum}..{maximum}cm"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--message", choices=("distance_sensor", "obstacle_distance"), default="distance_sensor")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=158)
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    if not 1 <= args.system_id <= 255 or not 1 <= args.component_id <= 255 or not 0 <= args.sensor_id <= 255:
        parser.error("system/component IDs must be 1..255 and sensor ID must be 0..255")
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    wanted = 132 if args.message == "distance_sensor" else 330
    heartbeat = False
    reading = None
    deadline = time.monotonic() + args.timeout

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((args.address, args.port))
            while time.monotonic() < deadline and (not heartbeat or reading is None):
                sock.settimeout(max(0.01, deadline - time.monotonic()))
                message_id, system_id, component_id, payload = parse_packet(sock.recv(512))
                if (system_id, component_id) != (args.system_id, args.component_id):
                    continue
                if message_id == 0:
                    if len(payload) < 9 or payload[4] != 18 or payload[5] != 8:
                        raise ValueError("unexpected HEARTBEAT vehicle or autopilot type")
                    heartbeat = True
                elif message_id == wanted:
                    reading = describe_range(message_id, payload, args.sensor_id)
    except (OSError, ValueError) as exc:
        print(f"MAVLink rangefinder check failed: {exc}", file=sys.stderr)
        return 1

    if not heartbeat or reading is None:
        missing = "heartbeat" if not heartbeat else args.message
        print(f"MAVLink rangefinder check failed: no {missing} within {args.timeout:g}s", file=sys.stderr)
        return 1
    print(f"MAVLink rangefinder OK: heartbeat system={args.system_id} component={args.component_id}; {reading}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
