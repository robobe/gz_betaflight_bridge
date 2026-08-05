from __future__ import annotations

import argparse
from dataclasses import dataclass

from .mission import MissionConfig


@dataclass(frozen=True)
class AppConfig:
    host: str
    port: int
    timeout_s: float
    mission: MissionConfig


def parse_args(argv: list[str] | None = None) -> AppConfig:
    parser = argparse.ArgumentParser(
        description="Arm, climb 3 m, yaw CW 180 degrees, return CCW, land, and disarm through MSP."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=25.0)
    parser.add_argument("--target-height", type=float, default=3.0)
    parser.add_argument("--takeoff-step", type=float, default=0.5)
    parser.add_argument("--step-dwell", type=float, default=0.75)
    parser.add_argument("--step-speed", type=float, default=0.30)
    parser.add_argument("--descent-rate", type=float, default=1.0)
    parser.add_argument("--hover-throttle", type=int, default=1660)
    parser.add_argument("--altitude-kp", type=float, default=20.0)
    parser.add_argument("--altitude-ki", type=float, default=10.0)
    parser.add_argument("--integral-limit", type=float, default=8.0)
    parser.add_argument("--altitude-kd", type=float, default=30.0)
    parser.add_argument("--min-throttle", type=int, default=1300)
    parser.add_argument("--max-throttle", type=int, default=1850)
    parser.add_argument("--throttle-slew-rate", type=float, default=1000.0)
    parser.add_argument("--yaw-max-offset", type=int, default=60)
    parser.add_argument("--yaw-min-offset", type=int, default=20)
    parser.add_argument("--yaw-slew-rate", type=float, default=60.0)
    parser.add_argument(
        "--reverse-yaw",
        action="store_true",
        help="Use RC values below 1500 for clockwise rotation.",
    )
    parser.add_argument("--yaw-timeout", type=float, default=40.0)
    parser.add_argument("--takeoff-timeout", type=float, default=20.0)
    parser.add_argument("--descent-timeout", type=float, default=10.0)
    parser.add_argument("--disarm-burst", type=float, default=1.0)
    args = parser.parse_args(argv)

    mission = MissionConfig(
        rate_hz=args.rate,
        target_height_m=args.target_height,
        takeoff_step_height_m=args.takeoff_step,
        takeoff_step_dwell_s=args.step_dwell,
        takeoff_step_speed_mps=args.step_speed,
        descent_rate_mps=args.descent_rate,
        hover_throttle=args.hover_throttle,
        altitude_kp=args.altitude_kp,
        altitude_ki=args.altitude_ki,
        altitude_kd=args.altitude_kd,
        integral_limit_m_s=args.integral_limit,
        min_throttle=args.min_throttle,
        max_throttle=args.max_throttle,
        throttle_slew_rate_pwm_s=args.throttle_slew_rate,
        yaw_max_offset_pwm=args.yaw_max_offset,
        yaw_min_offset_pwm=args.yaw_min_offset,
        yaw_slew_rate_pwm_s=args.yaw_slew_rate,
        yaw_clockwise_pwm_sign=-1 if args.reverse_yaw else 1,
        yaw_timeout_s=args.yaw_timeout,
        takeoff_timeout_s=args.takeoff_timeout,
        descent_timeout_s=args.descent_timeout,
        disarm_burst_s=args.disarm_burst,
    )
    return AppConfig(args.host, args.port, args.timeout, mission)
