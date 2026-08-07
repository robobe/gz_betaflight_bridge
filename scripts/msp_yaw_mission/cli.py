from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .controller import YawMissionConfig


DEFAULT_CONFIG_PATH = Path(__file__).with_name("msp_yaw_mission.yaml")


@dataclass(frozen=True)
class AppConfig:
    host: str
    port: int
    timeout_s: float
    mission: YawMissionConfig
    log_directory: Path
    csv_flush_period_s: float


def _scalar(text: str) -> Any:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text.strip('"\'')


def _load_config(path: Path) -> dict[str, Any]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError as exc:
        raise ValueError(f"Yaw mission configuration not found: {path}") from exc
    document: dict[str, Any] = {}
    section: dict[str, Any] | None = None
    for line_number, original in enumerate(lines, 1):
        content = original.split("#", 1)[0].rstrip()
        if not content.strip():
            continue
        indentation = len(content) - len(content.lstrip(" "))
        stripped = content.strip()
        if indentation == 0 and stripped.endswith(":"):
            section = {}
            document[stripped[:-1].strip()] = section
        elif indentation == 2 and section is not None and ":" in stripped:
            key, value = stripped.split(":", 1)
            section[key.strip()] = _scalar(value.strip())
        else:
            raise ValueError(f"Unsupported YAML structure in {path}:{line_number}")
    return document


def _section(document: dict[str, Any], name: str) -> dict[str, Any]:
    value = document.get(name, {})
    if not isinstance(value, dict):
        raise ValueError(f"Yaw configuration section '{name}' must be a mapping")
    return value


def _pick(cli_value: Any, section: dict[str, Any], key: str, fallback: Any) -> Any:
    return cli_value if cli_value is not None else section.get(key, fallback)


def parse_args(argv: list[str] | None = None) -> AppConfig:
    parser = argparse.ArgumentParser(
        description=(
            "Take off, hold altitude, yaw CCW 180 degrees at 15 deg/s, return CW, "
            "land, and disarm through MSP."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--target-altitude", type=float)
    parser.add_argument("--yaw-rate", type=float)
    parser.add_argument("--yaw-angle", type=float)
    parser.add_argument("--hover-throttle", type=int)
    parser.add_argument("--kp", type=float)
    parser.add_argument("--ki", type=float)
    parser.add_argument("--kd", type=float)
    parser.add_argument("--log-directory", type=Path)
    parser.add_argument("--reverse-yaw", action="store_true", default=None)
    args = parser.parse_args(argv)

    try:
        document = _load_config(args.config)
        connection = _section(document, "connection")
        control = _section(document, "control")
        mission_section = _section(document, "mission")
        takeoff = _section(document, "takeoff")
        hover = _section(document, "hover")
        yaw = _section(document, "yaw")
        landing = _section(document, "landing")
        safety = _section(document, "safety")
        logging = _section(document, "logging")
        reverse_yaw = bool(_pick(args.reverse_yaw, yaw, "reverse_yaw", False))
        mission = YawMissionConfig(
            target_altitude_m=float(
                _pick(args.target_altitude, mission_section, "target_altitude_m", 5.0)
            ),
            rate_hz=float(mission_section.get("rate_hz", 25.0)),
            duration_s=1.0,
            hover_throttle=int(_pick(args.hover_throttle, control, "hover_throttle", 1660)),
            kp=float(_pick(args.kp, control, "kp", 20.0)),
            ki=float(_pick(args.ki, control, "ki", 10.0)),
            kd=float(_pick(args.kd, control, "kd", 30.0)),
            velocity_filter_time_constant_s=float(
                control.get("velocity_filter_time_constant_s", 0.25)
            ),
            integral_limit_m_s=float(control.get("integral_limit_m_s", 8.0)),
            min_throttle=int(control.get("min_throttle", 1300)),
            max_throttle=int(control.get("max_throttle", 1850)),
            throttle_slew_rate_pwm_s=float(
                control.get("throttle_slew_rate_pwm_s", 1000.0)
            ),
            integral_gate_error_m=float(control.get("integral_gate_error_m", 0.30)),
            integral_gate_speed_mps=float(control.get("integral_gate_speed_mps", 0.50)),
            prearm_duration_s=float(mission_section.get("prearm_duration_s", 3.0)),
            low_throttle=int(mission_section.get("low_throttle", 1000)),
            disarm_burst_s=float(mission_section.get("disarm_burst_s", 1.0)),
            angle_mode=bool(mission_section.get("angle_mode", True)),
            arming_timeout_s=float(safety.get("arming_timeout_s", 5.0)),
            telemetry_stale_s=float(safety.get("telemetry_stale_s", 0.30)),
            status_period_s=float(safety.get("status_period_s", 0.2)),
            liftoff_height_m=float(takeoff.get("liftoff_height_m", 0.05)),
            liftoff_speed_mps=float(takeoff.get("liftoff_speed_mps", 0.10)),
            liftoff_timeout_s=float(takeoff.get("liftoff_timeout_s", 3.0)),
            takeoff_climb_rate_mps=float(takeoff.get("climb_rate_mps", 1.0)),
            takeoff_climb_feedforward_pwm=int(takeoff.get("climb_feedforward_pwm", 30)),
            takeoff_max_lag_m=float(takeoff.get("max_lag_m", 0.75)),
            takeoff_ready_dwell_s=float(takeoff.get("ready_dwell_s", 0.0)),
            takeoff_ready_speed_mps=float(takeoff.get("ready_speed_mps", 1.0)),
            takeoff_timeout_s=float(takeoff.get("timeout_s", 20.0)),
            altitude_tolerance_m=float(hover.get("altitude_tolerance_m", 0.15)),
            settle_duration_s=float(hover.get("settle_duration_s", 1.0)),
            settle_timeout_s=float(hover.get("settle_timeout_s", 10.0)),
            yaw_angle_deg=float(_pick(args.yaw_angle, yaw, "angle_deg", 180.0)),
            yaw_rate_dps=float(_pick(args.yaw_rate, yaw, "rate_dps", 15.0)),
            yaw_rate_feedforward_pwm=int(yaw.get("rate_feedforward_pwm", 20)),
            yaw_rate_kp_pwm_per_dps=float(yaw.get("rate_kp_pwm_per_dps", 2.0)),
            yaw_rate_filter_time_constant_s=float(
                yaw.get("rate_filter_time_constant_s", 0.15)
            ),
            yaw_max_offset_pwm=int(yaw.get("max_offset_pwm", 60)),
            yaw_slew_rate_pwm_s=float(yaw.get("slew_rate_pwm_s", 60.0)),
            yaw_slow_zone_deg=float(yaw.get("slow_zone_deg", 60.0)),
            yaw_tolerance_deg=float(yaw.get("tolerance_deg", 5.0)),
            yaw_settle_s=float(yaw.get("settle_s", 0.5)),
            yaw_timeout_s=float(yaw.get("timeout_s", 40.0)),
            yaw_altitude_gate_error_m=float(yaw.get("altitude_gate_error_m", 0.30)),
            yaw_altitude_gate_speed_mps=float(yaw.get("altitude_gate_speed_mps", 0.50)),
            max_yaw_altitude_error_m=float(yaw.get("max_altitude_error_m", 0.75)),
            yaw_clockwise_pwm_sign=(
                -1 if reverse_yaw else int(yaw.get("clockwise_pwm_sign", 1))
            ),
            descent_rate_mps=float(landing.get("descent_rate_mps", 1.0)),
            landing_height_m=float(landing.get("height_m", 0.15)),
            landing_speed_mps=float(landing.get("speed_mps", 0.15)),
            landing_settle_s=float(landing.get("settle_s", 0.5)),
            landing_timeout_s=float(landing.get("timeout_s", 20.0)),
            log_period_s=float(logging.get("console_period_s", 0.5)),
        )
        port = int(_pick(args.port, connection, "port", 5761))
        timeout_s = float(_pick(args.timeout, connection, "timeout_s", 1.0))
        csv_flush_period_s = float(logging.get("csv_flush_period_s", 1.0))
        if not 1 <= port <= 65535:
            raise ValueError("connection port must be between 1 and 65535")
        if timeout_s <= 0.0 or csv_flush_period_s <= 0.0:
            raise ValueError("connection timeout and CSV flush period must be positive")
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))

    return AppConfig(
        host=str(_pick(args.host, connection, "host", "127.0.0.1")),
        port=port,
        timeout_s=timeout_s,
        mission=mission,
        log_directory=Path(
            _pick(args.log_directory, logging, "directory", "logs/msp-yaw")
        ),
        csv_flush_period_s=csv_flush_period_s,
    )
