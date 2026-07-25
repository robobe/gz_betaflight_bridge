from __future__ import annotations

import argparse
from dataclasses import dataclass

from .altitude import AltitudeController
from .mission import MissionConfig
from .position import PositionController


@dataclass(frozen=True)
class AppConfig:
    host: str
    port: int
    timeout_s: float
    pose_topic: str
    model_name: str
    mission: MissionConfig
    altitude: AltitudeController
    position: PositionController


def parse_args(argv: list[str] | None = None) -> AppConfig:
    parser = argparse.ArgumentParser(description="MSP square mission controller for Betaflight SITL.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--pose-topic", default="/world/quadcopter/dynamic_pose/info")
    parser.add_argument("--model-name", default="X3")
    parser.add_argument("--takeoff-altitude", type=float, default=4.0)
    parser.add_argument(
        "--start-square-altitude",
        type=float,
        default=0.0,
        help=(
            "Optional altitude gate for starting square legs before takeoff altitude settles. "
            "Default 0 waits for --takeoff-altitude within --position-tolerance."
        ),
    )
    parser.add_argument("--square-side", type=float, default=6.0)
    parser.add_argument("--max-horizontal-speed", type=float, default=1.0)
    parser.add_argument("--descent-rate", type=float, default=1.0)
    parser.add_argument("--position-tolerance", type=float, default=0.5)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--pose-timeout", type=float, default=10.0)
    parser.add_argument("--max-leg-duration", type=float, default=45.0)
    parser.add_argument("--max-mission-duration", type=float, default=240.0)
    parser.add_argument("--prearm-duration", type=float, default=3.0)
    parser.add_argument("--arm-low-duration", type=float, default=1.0)
    parser.add_argument("--log-period", type=float, default=1.0)
    parser.add_argument("--disarm-burst", type=float, default=1.0)
    parser.add_argument("--landing-altitude", type=float, default=0.20)
    parser.add_argument("--hover-throttle", type=int, default=1600)
    parser.add_argument("--kp-altitude", type=float, default=100.0)
    parser.add_argument("--kd-altitude", type=float, default=90.0)
    parser.add_argument("--min-throttle", type=int, default=1100)
    parser.add_argument("--max-throttle", type=int, default=2000)
    parser.add_argument("--kp-position", type=float, default=0.8)
    parser.add_argument("--ki-position", type=float, default=0.05)
    parser.add_argument("--kd-position", type=float, default=0.35)
    parser.add_argument("--position-integral-limit", type=float, default=3.0)
    parser.add_argument("--rc-us-per-mps", type=float, default=250.0)
    parser.add_argument("--roll-min", type=int, default=1200)
    parser.add_argument("--roll-max", type=int, default=1800)
    parser.add_argument("--pitch-min", type=int, default=1200)
    parser.add_argument("--pitch-max", type=int, default=1800)
    parser.add_argument("--pitch-forward-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--roll-right-sign", type=int, choices=(-1, 1), default=1)
    angle_group = parser.add_mutually_exclusive_group()
    angle_group.add_argument("--angle-mode", dest="angle_mode", action="store_true", default=True)
    angle_group.add_argument("--no-angle-mode", dest="angle_mode", action="store_false")
    args = parser.parse_args(argv)

    mission = MissionConfig(
        takeoff_altitude_m=args.takeoff_altitude,
        start_square_altitude_m=args.start_square_altitude,
        square_side_m=args.square_side,
        descent_rate_mps=args.descent_rate,
        landing_altitude_m=args.landing_altitude,
        position_tolerance_m=args.position_tolerance,
        rate_hz=args.rate,
        pose_timeout_s=args.pose_timeout,
        max_leg_duration_s=args.max_leg_duration,
        max_mission_duration_s=args.max_mission_duration,
        prearm_duration_s=args.prearm_duration,
        arm_low_duration_s=args.arm_low_duration,
        log_period_s=args.log_period,
        disarm_burst_s=args.disarm_burst,
        angle_mode=args.angle_mode,
    )
    altitude = AltitudeController(
        hover_throttle=args.hover_throttle,
        kp=args.kp_altitude,
        kd=args.kd_altitude,
        min_throttle=args.min_throttle,
        max_throttle=args.max_throttle,
    )
    position = PositionController(
        kp_position=args.kp_position,
        ki_position=args.ki_position,
        kd_velocity=args.kd_position,
        integral_limit=args.position_integral_limit,
        max_horizontal_speed_mps=args.max_horizontal_speed,
        rc_us_per_mps=args.rc_us_per_mps,
        roll_min=args.roll_min,
        roll_max=args.roll_max,
        pitch_min=args.pitch_min,
        pitch_max=args.pitch_max,
        pitch_forward_sign=args.pitch_forward_sign,
        roll_right_sign=args.roll_right_sign,
    )
    return AppConfig(
        host=args.host,
        port=args.port,
        timeout_s=args.timeout,
        pose_topic=args.pose_topic,
        model_name=args.model_name,
        mission=mission,
        altitude=altitude,
        position=position,
    )
