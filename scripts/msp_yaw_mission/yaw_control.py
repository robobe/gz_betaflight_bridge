from __future__ import annotations

from dataclasses import dataclass


class YawUnwrapper:
    def __init__(self) -> None:
        self._previous_wrapped_deg: float | None = None
        self._unwrapped_deg = 0.0

    def update(self, wrapped_deg: float) -> float:
        if self._previous_wrapped_deg is None:
            self._previous_wrapped_deg = wrapped_deg
            self._unwrapped_deg = wrapped_deg
            return self._unwrapped_deg
        delta_deg = (wrapped_deg - self._previous_wrapped_deg + 180.0) % 360.0 - 180.0
        self._unwrapped_deg += delta_deg
        self._previous_wrapped_deg = wrapped_deg
        return self._unwrapped_deg


class YawRateEstimator:
    def __init__(self, time_constant_s: float = 0.15) -> None:
        if time_constant_s < 0.0:
            raise ValueError("time_constant_s must not be negative")
        self._time_constant_s = time_constant_s
        self._previous_heading_deg: float | None = None
        self._previous_timestamp_s: float | None = None
        self._filtered_rate_dps = 0.0

    def update(self, heading_deg: float, timestamp_s: float) -> tuple[float, float]:
        previous_heading_deg = self._previous_heading_deg
        previous_timestamp_s = self._previous_timestamp_s
        self._previous_heading_deg = heading_deg
        self._previous_timestamp_s = timestamp_s
        if previous_heading_deg is None or previous_timestamp_s is None:
            return self._filtered_rate_dps, 0.0
        dt_s = timestamp_s - previous_timestamp_s
        if dt_s <= 0.0:
            return self._filtered_rate_dps, 0.0
        measured_rate_dps = (heading_deg - previous_heading_deg) / dt_s
        alpha = 1.0 if self._time_constant_s == 0.0 else dt_s / (self._time_constant_s + dt_s)
        self._filtered_rate_dps += alpha * (measured_rate_dps - self._filtered_rate_dps)
        return self._filtered_rate_dps, dt_s


@dataclass(frozen=True)
class YawRateController:
    target_rate_dps: float = 15.0
    feedforward_offset_pwm: int = 20
    rate_kp_pwm_per_dps: float = 2.0
    max_offset_pwm: int = 60
    slow_zone_deg: float = 60.0
    tolerance_deg: float = 5.0
    center_pwm: int = 1500

    def command(
        self,
        target_deg: float,
        heading_deg: float,
        measured_rate_dps: float,
        direction: int,
    ) -> int:
        if direction not in (-1, 1):
            raise ValueError("direction must be -1 (CCW) or +1 (CW)")
        remaining_deg = direction * (target_deg - heading_deg)
        if remaining_deg <= self.tolerance_deg:
            return self.center_pwm
        rate_scale = min(1.0, remaining_deg / self.slow_zone_deg)
        desired_rate_dps = self.target_rate_dps * rate_scale
        measured_along_direction_dps = direction * measured_rate_dps
        offset_pwm = (
            self.feedforward_offset_pwm * rate_scale
            + self.rate_kp_pwm_per_dps * (desired_rate_dps - measured_along_direction_dps)
        )
        bounded_offset_pwm = max(0, min(self.max_offset_pwm, int(round(offset_pwm))))
        return self.center_pwm + direction * bounded_offset_pwm
