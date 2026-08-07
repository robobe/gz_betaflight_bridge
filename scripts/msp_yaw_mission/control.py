from __future__ import annotations

from dataclasses import dataclass

from flight_control.altitude import (
    AltitudePid as AltitudePid,
    ThrottleSlewLimiter as ThrottleSlewLimiter,
    VerticalVelocityEstimator as VerticalVelocityEstimator,
    altitude_steps as altitude_steps,
)


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


@dataclass(frozen=True)
class DirectedYawController:
    center_pwm: int = 1500
    max_offset_pwm: int = 200
    min_offset_pwm: int = 60
    slow_zone_deg: float = 45.0
    tolerance_deg: float = 5.0

    def command(self, target_deg: float, heading_deg: float, direction: int) -> int:
        if direction not in (-1, 1):
            raise ValueError("direction must be -1 (CCW) or +1 (CW)")
        remaining_deg = direction * (target_deg - heading_deg)
        if remaining_deg <= self.tolerance_deg:
            return self.center_pwm
        scaled_offset = self.max_offset_pwm * min(1.0, remaining_deg / self.slow_zone_deg)
        offset = max(self.min_offset_pwm, int(round(scaled_offset)))
        return self.center_pwm + direction * offset
