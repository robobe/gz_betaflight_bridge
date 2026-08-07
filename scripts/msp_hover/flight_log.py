from __future__ import annotations

import json
import math
from dataclasses import asdict, fields
from datetime import datetime, timezone
from pathlib import Path

from flight_log.csv_recorder import CsvRecorder

from .controller import HoverConfig, HoverCycle, Phase


class HoverFlightLog:
    """Hover-specific online metrics backed by the reusable CSV recorder."""

    def __init__(
        self,
        directory: Path,
        config: HoverConfig,
        *,
        csv_flush_period_s: float = 1.0,
        oscillation_deadband_m: float = 0.05,
    ) -> None:
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        self.csv_path = directory / f"hover-{run_id}.csv"
        self.summary_path = directory / f"hover-{run_id}-summary.json"
        self._config = config
        self._csv = CsvRecorder(
            self.csv_path,
            [field.name for field in fields(HoverCycle)],
            flush_period_s=csv_flush_period_s,
        )
        self._deadband_m = oscillation_deadband_m
        self._count = 0
        self._error_sum = 0.0
        self._abs_error_sum = 0.0
        self._squared_error_sum = 0.0
        self._squared_velocity_sum = 0.0
        self._max_abs_error = 0.0
        self._overshoot = 0.0
        self._throttle_sum = 0
        self._saturation_count = 0
        self._crossings = 0
        self._error_side = 0
        self._score_first_s: float | None = None
        self._score_last_s: float | None = None
        self._takeoff_first_s: float | None = None
        self._takeoff_last_s: float | None = None
        self._takeoff_peak_target_m = 0.0
        self._takeoff_max_abs_error = 0.0
        self._landing_first_s: float | None = None
        self._landing_last_s: float | None = None
        self._landing_max_descent_speed = 0.0
        self._landing_confirmed = False
        self._failure_reason: str | None = None
        self._closed = False

    def record(self, cycle: HoverCycle) -> None:
        self._csv.write(asdict(cycle))
        if cycle.phase == Phase.TAKEOFF.value:
            self._takeoff_first_s = cycle.elapsed_s if self._takeoff_first_s is None else self._takeoff_first_s
            self._takeoff_last_s = cycle.elapsed_s
            self._takeoff_peak_target_m = max(
                self._takeoff_peak_target_m, cycle.target_altitude_m
            )
            self._takeoff_max_abs_error = max(self._takeoff_max_abs_error, abs(cycle.altitude_error_m))
        elif cycle.phase == Phase.SCORED_HOVER.value:
            self._record_scored_hover(cycle)
        elif cycle.phase in (Phase.DESCEND.value, Phase.ABORT_DESCEND.value):
            self._landing_first_s = cycle.elapsed_s if self._landing_first_s is None else self._landing_first_s
            self._landing_last_s = cycle.elapsed_s
            self._landing_max_descent_speed = max(
                self._landing_max_descent_speed,
                max(0.0, -cycle.control_velocity_mps),
            )

    def _record_scored_hover(self, cycle: HoverCycle) -> None:
        error = cycle.altitude_error_m
        self._count += 1
        self._error_sum += error
        self._abs_error_sum += abs(error)
        self._squared_error_sum += error * error
        self._squared_velocity_sum += cycle.control_velocity_mps * cycle.control_velocity_mps
        self._max_abs_error = max(self._max_abs_error, abs(error))
        self._overshoot = max(self._overshoot, -error)
        self._throttle_sum += cycle.sent_throttle_pwm
        self._saturation_count += cycle.sent_throttle_pwm in (
            self._config.min_throttle,
            self._config.max_throttle,
        )
        side = 1 if error > self._deadband_m else -1 if error < -self._deadband_m else 0
        if side and self._error_side and side != self._error_side:
            self._crossings += 1
        if side:
            self._error_side = side
        self._score_first_s = cycle.elapsed_s if self._score_first_s is None else self._score_first_s
        self._score_last_s = cycle.elapsed_s

    def mark_landing_confirmed(self) -> None:
        self._landing_confirmed = True

    def mark_failure(self, reason: str) -> None:
        self._failure_reason = reason

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._csv.close()
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.write_text(json.dumps(self._summary(), indent=2) + "\n", encoding="utf-8")
        print(f"flight log: {self.csv_path}", flush=True)
        print(f"tuning summary: {self.summary_path}", flush=True)

    @staticmethod
    def _duration(first_s: float | None, last_s: float | None) -> float:
        return 0.0 if first_s is None or last_s is None else max(0.0, last_s - first_s)

    def _summary(self) -> dict[str, object]:
        duration_s = self._duration(self._score_first_s, self._score_last_s)
        return {
            "control": {
                "hover_throttle": self._config.hover_throttle,
                "kp": self._config.kp,
                "ki": self._config.ki,
                "kd": self._config.kd,
                "velocity_filter_time_constant_s": self._config.velocity_filter_time_constant_s,
                "integral_limit_m_s": self._config.integral_limit_m_s,
                "min_throttle": self._config.min_throttle,
                "max_throttle": self._config.max_throttle,
            },
            "takeoff": {
                "duration_s": self._duration(self._takeoff_first_s, self._takeoff_last_s),
                "peak_target_m": self._takeoff_peak_target_m,
                "max_step_error_m": self._takeoff_max_abs_error,
            },
            "steady_hover": {
                "samples": self._count,
                "duration_s": duration_s,
                "mean_error_m": self._error_sum / self._count if self._count else None,
                "mae_m": self._abs_error_sum / self._count if self._count else None,
                "rmse_m": math.sqrt(self._squared_error_sum / self._count) if self._count else None,
                "max_abs_error_m": self._max_abs_error if self._count else None,
                "overshoot_m": self._overshoot if self._count else None,
                "vertical_speed_rms_mps": (
                    math.sqrt(self._squared_velocity_sum / self._count) if self._count else None
                ),
                "mean_throttle_pwm": self._throttle_sum / self._count if self._count else None,
                "throttle_saturation_percent": (
                    100.0 * self._saturation_count / self._count if self._count else None
                ),
                "oscillations_per_minute": (
                    60.0 * self._crossings / duration_s if duration_s > 0.0 else None
                ),
                "oscillation_deadband_m": self._deadband_m,
            },
            "landing": {
                "duration_s": self._duration(self._landing_first_s, self._landing_last_s),
                "max_descent_speed_mps": self._landing_max_descent_speed,
                "confirmed": self._landing_confirmed,
            },
            "safety": {
                "passed": self._failure_reason is None and self._landing_confirmed,
                "failure_reason": self._failure_reason,
            },
        }

    def __enter__(self) -> HoverFlightLog:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        if exc is not None and self._failure_reason is None:
            self.mark_failure(str(exc))
        self.close()
