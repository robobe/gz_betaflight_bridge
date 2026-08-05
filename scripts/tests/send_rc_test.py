#!/usr/bin/env python3
"""Send simple Betaflight SITL RC packets to UDP 9004."""

import argparse
import socket
import struct
import time


RC_PACKET_FORMAT = "<d16H"


def channels(throttle: int, arm: bool, angle: bool) -> list[int]:
    values = [1500] * 16
    values[0] = 1500  # roll
    values[1] = 1500  # pitch
    values[2] = throttle
    values[3] = 1500  # yaw
    values[4] = 2000 if arm else 1000  # AUX1
    values[5] = 2000 if angle else 1000  # AUX2
    return values


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9004)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--angle", action="store_true")
    parser.add_argument("--throttle", type=int, default=1000)
    parser.add_argument("--throttle-ramp", action="store_true")
    parser.add_argument("--ramp-end", type=int, default=2000)
    parser.add_argument("--takeoff-sequence", action="store_true")
    parser.add_argument("--disarm-duration", type=float, default=3.0)
    parser.add_argument("--arm-low-duration", type=float, default=5.0)
    parser.add_argument("--ramp-duration", type=float, default=10.0)
    parser.add_argument("--hold-duration", type=float, default=5.0)
    args = parser.parse_args()

    period = 1.0 / args.rate
    start = time.monotonic()
    total_duration = args.duration
    if args.takeoff_sequence:
        total_duration = (
            args.disarm_duration
            + args.arm_low_duration
            + args.ramp_duration
            + args.hold_duration
        )
    end = start + total_duration

    last_phase = None

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while time.monotonic() < end:
            elapsed = time.monotonic() - start
            arm = args.arm
            angle = args.angle
            throttle = args.throttle
            phase = "manual"

            if args.takeoff_sequence:
                angle = True
                if elapsed < args.disarm_duration:
                    arm = False
                    throttle = 1000
                    phase = "disarmed-low-throttle"
                elif elapsed < args.disarm_duration + args.arm_low_duration:
                    arm = True
                    throttle = 1000
                    phase = "armed-low-throttle"
                elif elapsed < args.disarm_duration + args.arm_low_duration + args.ramp_duration:
                    arm = True
                    ramp_elapsed = elapsed - args.disarm_duration - args.arm_low_duration
                    ratio = min(1.0, ramp_elapsed / args.ramp_duration)
                    throttle = int(args.throttle + ratio * (args.ramp_end - args.throttle))
                    phase = "armed-throttle-ramp"
                else:
                    arm = True
                    throttle = args.ramp_end
                    phase = "armed-hold"
            elif args.throttle_ramp:
                ratio = min(1.0, elapsed / args.duration)
                throttle = int(args.throttle + ratio * (args.ramp_end - args.throttle))
                phase = "manual-throttle-ramp"

            if phase != last_phase:
                print(
                    f"{phase}: throttle={throttle} arm={int(arm)} angle={int(angle)}",
                    flush=True,
                )
                last_phase = phase

            packet = struct.pack(RC_PACKET_FORMAT, elapsed, *channels(throttle, arm, angle))
            sock.sendto(packet, (args.ip, args.port))
            time.sleep(period)


if __name__ == "__main__":
    main()
