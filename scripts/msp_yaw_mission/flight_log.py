from __future__ import annotations

import json
import math
from dataclasses import asdict, fields
from datetime import datetime, timezone
from pathlib import Path

from flight_log.csv_recorder import CsvRecorder

from .controller import Phase, YawCycle, YawMissionConfig


class YawFlightLog:
    """Full-rate yaw mission recording with bounded online metrics."""

    def __init__(
        self,
        directory: Path,
        config: YawMissionConfig,
        *,
        csv_flush_period_s: float = 1.0,
    ) -> None:
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        self.csv_path = directory / f"yaw-{run_id}.csv"
        self.summary_path = directory / f"yaw-{run_id}-summary.json"
        self._config = config
        self._csv = CsvRecorder(
            self.csv_path,
            [field.name for field in fields(YawCycle)],
            flush_period_s=csv_flush_period_s,
        )
        self._yaw_samples = 0
        self._yaw_rate_error_squared_sum = 0.0
        self._yaw_rate_max_dps = 0.0
        self._yaw_altitude_max_error_m = 0.0
        self._phases: set[str] = set()
        self._landing_confirmed = False
        self._failure_reason: str | None = None
        self._closed = False

    def record(self, cycle: YawCycle) -> None:
        self._csv.write(asdict(cycle))
        self._phases.add(cycle.phase)
        if cycle.phase in (Phase.YAW_CCW.value, Phase.YAW_CW.value):
            measured_rate = abs(cycle.yaw_rate_dps)
            self._yaw_samples += 1
            self._yaw_rate_error_squared_sum += (
                measured_rate - self._config.yaw_rate_dps
            ) ** 2
            self._yaw_rate_max_dps = max(self._yaw_rate_max_dps, measured_rate)
            self._yaw_altitude_max_error_m = max(
                self._yaw_altitude_max_error_m, abs(cycle.altitude_error_m)
            )

    def mark_landing_confirmed(self) -> None:
        self._landing_confirmed = True

    def mark_failure(self, reason: str) -> None:
        self._failure_reason = reason

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._csv.close()
        rate_rmse = (
            math.sqrt(self._yaw_rate_error_squared_sum / self._yaw_samples)
            if self._yaw_samples
            else None
        )
        summary = {
            "mission": {
                "target_altitude_m": self._config.target_altitude_m,
                "yaw_angle_deg": self._config.yaw_angle_deg,
                "yaw_rate_dps": self._config.yaw_rate_dps,
                "order": ["ccw", "cw_home"],
            },
            "yaw": {
                "samples": self._yaw_samples,
                "rate_rmse_dps": rate_rmse,
                "max_abs_rate_dps": self._yaw_rate_max_dps,
                "max_altitude_error_m": self._yaw_altitude_max_error_m,
            },
            "landing": {"confirmed": self._landing_confirmed},
            "safety": {
                "passed": self._failure_reason is None and self._landing_confirmed,
                "failure_reason": self._failure_reason,
            },
            "phases": sorted(self._phases),
        }
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"flight log: {self.csv_path}", flush=True)
        print(f"mission summary: {self.summary_path}", flush=True)

    def __enter__(self) -> YawFlightLog:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        if exc is not None and self._failure_reason is None:
            self.mark_failure(str(exc))
        self.close()
