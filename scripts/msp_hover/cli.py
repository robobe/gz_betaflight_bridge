from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .controller import HoverConfig


DEFAULT_CONFIG_PATH = Path(__file__).with_name("msp_hover.yaml")


@dataclass(frozen=True)
class AppConfig:
    host: str
    port: int
    timeout_s: float
    hover: HoverConfig
    log_directory: Path
    csv_flush_period_s: float
    oscillation_deadband_m: float


def _scalar(text: str) -> Any:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text.strip('"\'')


def _load_config(path: Path) -> dict[str, Any]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError as exc:
        raise ValueError(f"Hover configuration not found: {path}") from exc
    document: dict[str, Any] = {}
    section: dict[str, Any] | None = None
    for line_number, original in enumerate(lines, 1):
        content = original.split("#", 1)[0].rstrip()
        if not content.strip():
            continue
        indentation = len(content) - len(content.lstrip(" "))
        stripped = content.strip()
        if indentation == 0 and stripped.endswith(":"):
            name = stripped[:-1].strip()
            section = {}
            document[name] = section
            continue
        if indentation == 2 and section is not None and ":" in stripped:
            key, value = stripped.split(":", 1)
            section[key.strip()] = _scalar(value.strip())
            continue
        raise ValueError(f"Unsupported YAML structure in {path}:{line_number}")
    return document


def _section(config: dict[str, Any], name: str) -> dict[str, Any]:
    value = config.get(name, {})
    if not isinstance(value, dict):
        raise ValueError(f"Hover configuration section '{name}' must be a mapping")
    return value


def _pick(cli_value: Any, section: dict[str, Any], key: str, fallback: Any) -> Any:
    return cli_value if cli_value is not None else section.get(key, fallback)


def parse_args(argv: list[str] | None = None) -> AppConfig:
    parser = argparse.ArgumentParser(description="Safe MSP hover PID tuning mission for Betaflight SITL.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--target-altitude", type=float)
    parser.add_argument("--rate", type=float)
    parser.add_argument("--duration", type=float, help="Seconds in the scored steady-hover phase.")
    parser.add_argument("--hover-throttle", type=int)
    parser.add_argument("--kp", type=float)
    parser.add_argument("--ki", type=float)
    parser.add_argument("--kd", type=float)
    parser.add_argument("--integral-limit", type=float)
    parser.add_argument("--min-throttle", type=int)
    parser.add_argument("--max-throttle", type=int)
    parser.add_argument("--prearm-duration", type=float)
    parser.add_argument("--log-period", type=float)
    parser.add_argument("--log-directory", type=Path)
    parser.add_argument("--disarm-burst", type=float)
    angle_group = parser.add_mutually_exclusive_group()
    angle_group.add_argument("--angle-mode", dest="angle_mode", action="store_true")
    angle_group.add_argument("--no-angle-mode", dest="angle_mode", action="store_false")
    parser.set_defaults(angle_mode=None)
    args = parser.parse_args(argv)

    try:
        document = _load_config(args.config)
        connection = _section(document, "connection")
        control = _section(document, "control")
        mission = _section(document, "mission")
        takeoff = _section(document, "takeoff")
        hover_test = _section(document, "hover_test")
        landing = _section(document, "landing")
        safety = _section(document, "safety")
        logging = _section(document, "logging")
        hover = HoverConfig(
            target_altitude_m=float(_pick(args.target_altitude, mission, "target_altitude_m", 3.0)),
            rate_hz=float(_pick(args.rate, mission, "rate_hz", 25.0)),
            duration_s=float(_pick(args.duration, hover_test, "duration_s", 10.0)),
            hover_throttle=int(_pick(args.hover_throttle, control, "hover_throttle", 1660)),
            kp=float(_pick(args.kp, control, "kp", 20.0)),
            ki=float(_pick(args.ki, control, "ki", 10.0)),
            kd=float(_pick(args.kd, control, "kd", 30.0)),
            velocity_filter_time_constant_s=float(
                control.get("velocity_filter_time_constant_s", 0.25)
            ),
            integral_limit_m_s=float(_pick(args.integral_limit, control, "integral_limit_m_s", 8.0)),
            min_throttle=int(_pick(args.min_throttle, control, "min_throttle", 1300)),
            max_throttle=int(_pick(args.max_throttle, control, "max_throttle", 1850)),
            throttle_slew_rate_pwm_s=float(control.get("throttle_slew_rate_pwm_s", 1000.0)),
            integral_gate_error_m=float(control.get("integral_gate_error_m", 0.30)),
            integral_gate_speed_mps=float(control.get("integral_gate_speed_mps", 0.50)),
            prearm_duration_s=float(_pick(args.prearm_duration, mission, "prearm_duration_s", 3.0)),
            arming_timeout_s=float(safety.get("arming_timeout_s", 5.0)),
            liftoff_height_m=float(takeoff.get("liftoff_height_m", 0.05)),
            liftoff_speed_mps=float(takeoff.get("liftoff_speed_mps", 0.10)),
            liftoff_timeout_s=float(takeoff.get("liftoff_timeout_s", 3.0)),
            takeoff_climb_rate_mps=float(takeoff.get("climb_rate_mps", 1.0)),
            takeoff_climb_feedforward_pwm=int(takeoff.get("climb_feedforward_pwm", 30)),
            takeoff_max_lag_m=float(takeoff.get("max_lag_m", 0.75)),
            takeoff_ready_dwell_s=float(takeoff.get("ready_dwell_s", 0.0)),
            takeoff_ready_speed_mps=float(takeoff.get("ready_speed_mps", 1.0)),
            takeoff_timeout_s=float(takeoff.get("timeout_s", 20.0)),
            altitude_tolerance_m=float(hover_test.get("altitude_tolerance_m", 0.15)),
            settle_duration_s=float(hover_test.get("settle_duration_s", 1.0)),
            settle_timeout_s=float(hover_test.get("settle_timeout_s", 10.0)),
            descent_rate_mps=float(landing.get("descent_rate_mps", 1.0)),
            landing_height_m=float(landing.get("height_m", 0.15)),
            landing_speed_mps=float(landing.get("speed_mps", 0.15)),
            landing_settle_s=float(landing.get("settle_s", 0.5)),
            landing_timeout_s=float(landing.get("timeout_s", 20.0)),
            max_altitude_error_m=float(safety.get("max_altitude_error_m", 1.0)),
            telemetry_stale_s=float(safety.get("telemetry_stale_s", 0.30)),
            status_period_s=float(safety.get("status_period_s", 0.2)),
            log_period_s=float(_pick(args.log_period, logging, "console_period_s", 0.5)),
            low_throttle=int(mission.get("low_throttle", 1000)),
            disarm_burst_s=float(_pick(args.disarm_burst, mission, "disarm_burst_s", 1.0)),
            angle_mode=bool(_pick(args.angle_mode, mission, "angle_mode", True)),
        )
        port = int(_pick(args.port, connection, "port", 5761))
        timeout_s = float(_pick(args.timeout, connection, "timeout_s", 1.0))
        csv_flush_period_s = float(logging.get("csv_flush_period_s", 1.0))
        oscillation_deadband_m = float(hover_test.get("oscillation_deadband_m", 0.05))
        if not 1 <= port <= 65535:
            raise ValueError("connection port must be between 1 and 65535")
        if timeout_s <= 0.0 or csv_flush_period_s <= 0.0 or oscillation_deadband_m < 0.0:
            raise ValueError("timeouts and flush period must be positive; deadband must not be negative")
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))

    return AppConfig(
        host=str(_pick(args.host, connection, "host", "127.0.0.1")),
        port=port,
        timeout_s=timeout_s,
        hover=hover,
        log_directory=Path(_pick(args.log_directory, logging, "directory", "logs/msp-hover")),
        csv_flush_period_s=csv_flush_period_s,
        oscillation_deadband_m=oscillation_deadband_m,
    )
