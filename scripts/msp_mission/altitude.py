from __future__ import annotations

from dataclasses import dataclass

from msp_hover.telemetry import AltitudeSample


@dataclass
class AltitudeController:
    hover_throttle: int
    kp: float
    kd: float
    min_throttle: int
    max_throttle: int
    _previous_sample: AltitudeSample | None = None

    def throttle(self, target_altitude_m: float, sample: AltitudeSample) -> tuple[int, float]:
        vertical_velocity_mps = self._derive_vertical_velocity(sample)
        error_m = target_altitude_m - sample.altitude_m
        throttle = self.hover_throttle + self.kp * error_m - self.kd * vertical_velocity_mps
        clamped = max(self.min_throttle, min(self.max_throttle, int(round(throttle))))
        return clamped, vertical_velocity_mps

    def _derive_vertical_velocity(self, sample: AltitudeSample) -> float:
        previous = self._previous_sample
        self._previous_sample = sample
        if previous is None:
            return sample.vertical_velocity_mps

        dt_s = sample.timestamp_s - previous.timestamp_s
        if dt_s <= 0.0:
            return sample.vertical_velocity_mps

        return (sample.altitude_m - previous.altitude_m) / dt_s
