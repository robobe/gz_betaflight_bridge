from __future__ import annotations

import argparse
from dataclasses import dataclass

from .controller import HoverConfig


@dataclass(frozen=True)
class AppConfig:
    host: str
    port: int
    timeout_s: float
    hover: HoverConfig


def parse_args(argv: list[str] | None = None) -> AppConfig:
    parser = argparse.ArgumentParser(description="MSP altitude-hold RC controller for Betaflight SITL.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--target-altitude", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until interrupted.")
    parser.add_argument("--hover-throttle", type=int, default=1600)
    parser.add_argument("--kp", type=float, default=100.0)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=90.0)
    parser.add_argument("--integral-limit", type=float, default=5.0)
    parser.add_argument("--min-throttle", type=int, default=1100)
    parser.add_argument("--max-throttle", type=int, default=2000)
    parser.add_argument("--prearm-duration", type=float, default=3.0)
    parser.add_argument("--arm-low-duration", type=float, default=1.0)
    parser.add_argument("--descent-duration", type=float, default=8.0)
    parser.add_argument("--landing-altitude", type=float, default=0.15)
    parser.add_argument("--log-period", type=float, default=1.0)
    parser.add_argument("--disarm-burst", type=float, default=1.0)
    angle_group = parser.add_mutually_exclusive_group()
    angle_group.add_argument("--angle-mode", dest="angle_mode", action="store_true", default=True)
    angle_group.add_argument("--no-angle-mode", dest="angle_mode", action="store_false")
    args = parser.parse_args(argv)

    hover = HoverConfig(
        target_altitude_m=args.target_altitude,
        rate_hz=args.rate,
        duration_s=args.duration,
        hover_throttle=args.hover_throttle,
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        integral_limit_m_s=args.integral_limit,
        min_throttle=args.min_throttle,
        max_throttle=args.max_throttle,
        prearm_duration_s=args.prearm_duration,
        arm_low_duration_s=args.arm_low_duration,
        descent_duration_s=args.descent_duration,
        landing_altitude_m=args.landing_altitude,
        log_period_s=args.log_period,
        disarm_burst_s=args.disarm_burst,
        angle_mode=args.angle_mode,
    )
    return AppConfig(host=args.host, port=args.port, timeout_s=args.timeout, hover=hover)
