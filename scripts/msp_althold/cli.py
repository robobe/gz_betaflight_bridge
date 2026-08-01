from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass(frozen=True)
class ScenarioConfig:
    host: str = "127.0.0.1"
    port: int = 5761
    timeout_s: float = 1.0
    rate_hz: float = 50.0
    takeoff_altitude_m: float = 3.0
    altitude_step_m: float = 1.0
    tolerance_m: float = 1.0
    settle_duration_s: float = 2.0
    hold_duration_s: float = 5.0
    phase_timeout_s: float = 30.0
    hover_throttle: int = 1750
    althold_throttle: int = 1700
    climb_throttle: int = 1850
    descent_throttle: int = 1450
    landing_duration_s: float = 8.0
    landing_altitude_m: float = 0.15
    prearm_duration_s: float = 3.0
    arm_low_duration_s: float = 1.0
    log_period_s: float = 0.5
    disarm_burst_s: float = 1.0


def parse_args(argv: list[str] | None = None) -> ScenarioConfig:
    parser = argparse.ArgumentParser(
        description="Exercise Betaflight's native ALT HOLD mode on AUX3 in SITL."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument("--altitude-step", type=float, default=1.0)
    parser.add_argument("--tolerance", type=float, default=1.0)
    parser.add_argument("--settle-duration", type=float, default=2.0)
    parser.add_argument("--hold-duration", type=float, default=5.0)
    parser.add_argument("--phase-timeout", type=float, default=30.0)
    parser.add_argument("--hover-throttle", type=int, default=1750)
    parser.add_argument("--althold-throttle", type=int, default=1700)
    parser.add_argument("--climb-throttle", type=int, default=1850)
    parser.add_argument("--descent-throttle", type=int, default=1450)
    parser.add_argument("--landing-duration", type=float, default=8.0)
    parser.add_argument("--landing-altitude", type=float, default=0.15)
    parser.add_argument("--prearm-duration", type=float, default=3.0)
    parser.add_argument("--arm-low-duration", type=float, default=1.0)
    parser.add_argument("--log-period", type=float, default=0.5)
    parser.add_argument("--disarm-burst", type=float, default=1.0)
    args = parser.parse_args(argv)

    positive = {
        "--rate": args.rate,
        "--takeoff-altitude": args.takeoff_altitude,
        "--altitude-step": args.altitude_step,
        "--tolerance": args.tolerance,
        "--settle-duration": args.settle_duration,
        "--hold-duration": args.hold_duration,
        "--phase-timeout": args.phase_timeout,
        "--landing-duration": args.landing_duration,
    }
    for name, value in positive.items():
        if value <= 0:
            parser.error(f"{name} must be greater than zero")
    for name in ("hover_throttle", "althold_throttle", "climb_throttle", "descent_throttle"):
        value = getattr(args, name)
        if not 800 <= value <= 2200:
            parser.error(f"--{name.replace('_', '-')} must be between 800 and 2200")
    if args.climb_throttle <= 1700:
        parser.error("--climb-throttle must be above the maximum captured hover throttle (1700)")
    if args.descent_throttle >= 1700:
        parser.error("--descent-throttle must be below the maximum captured hover throttle (1700)")
    if not 1100 <= args.althold_throttle <= 1700:
        parser.error("--althold-throttle must be within Betaflight's 1100 to 1700 hover range")

    return ScenarioConfig(
        host=args.host,
        port=args.port,
        timeout_s=args.timeout,
        rate_hz=args.rate,
        takeoff_altitude_m=args.takeoff_altitude,
        altitude_step_m=args.altitude_step,
        tolerance_m=args.tolerance,
        settle_duration_s=args.settle_duration,
        hold_duration_s=args.hold_duration,
        phase_timeout_s=args.phase_timeout,
        hover_throttle=args.hover_throttle,
        althold_throttle=args.althold_throttle,
        climb_throttle=args.climb_throttle,
        descent_throttle=args.descent_throttle,
        landing_duration_s=args.landing_duration,
        landing_altitude_m=args.landing_altitude,
        prearm_duration_s=args.prearm_duration,
        arm_low_duration_s=args.arm_low_duration,
        log_period_s=args.log_period,
        disarm_burst_s=args.disarm_burst,
    )
