from __future__ import annotations

from dataclasses import dataclass

from msp_core.telemetry import AltitudeSample


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class AltitudePid:
    hover_throttle: int
    kp: float
    ki: float
    kd: float
    min_throttle: int
    max_throttle: int
    integral_limit_m_s: float
    integral_error_m_s: float = 0.0

    def reset(self) -> None:
        self.integral_error_m_s = 0.0

    def command(
        self,
        target_m: float,
        altitude_m: float,
        vertical_velocity_mps: float,
        dt_s: float,
        *,
        integrate: bool = True,
    ) -> int:
        error_m = target_m - altitude_m
        candidate_integral = self.integral_error_m_s
        if integrate and dt_s > 0.0:
            candidate_integral = clamp(
                self.integral_error_m_s + error_m * dt_s,
                -self.integral_limit_m_s,
                self.integral_limit_m_s,
            )

        candidate_throttle = (
            self.hover_throttle
            + self.kp * error_m
            + self.ki * candidate_integral
            - self.kd * vertical_velocity_mps
        )
        drives_high_saturation = candidate_throttle > self.max_throttle and error_m > 0.0
        drives_low_saturation = candidate_throttle < self.min_throttle and error_m < 0.0
        if not drives_high_saturation and not drives_low_saturation:
            self.integral_error_m_s = candidate_integral

        throttle = (
            self.hover_throttle
            + self.kp * error_m
            + self.ki * self.integral_error_m_s
            - self.kd * vertical_velocity_mps
        )
        return int(round(clamp(throttle, self.min_throttle, self.max_throttle)))


@dataclass
class ThrottleSlewLimiter:
    rate_pwm_s: float
    current_pwm: float

    def reset(self, throttle_pwm: int) -> None:
        self.current_pwm = float(throttle_pwm)

    def command(self, desired_pwm: int, dt_s: float) -> int:
        if self.rate_pwm_s <= 0.0:
            raise ValueError("rate_pwm_s must be positive")
        if dt_s <= 0.0:
            return int(round(self.current_pwm))
        max_change = self.rate_pwm_s * dt_s
        change = clamp(desired_pwm - self.current_pwm, -max_change, max_change)
        self.current_pwm += change
        return int(round(self.current_pwm))


def altitude_steps(launch_altitude_m: float, target_height_m: float, step_height_m: float) -> list[float]:
    if target_height_m <= 0.0:
        raise ValueError("target_height_m must be positive")
    if step_height_m <= 0.0:
        raise ValueError("step_height_m must be positive")
    steps: list[float] = []
    height_m = step_height_m
    while height_m < target_height_m:
        steps.append(launch_altitude_m + height_m)
        height_m += step_height_m
    steps.append(launch_altitude_m + target_height_m)
    return steps


class VerticalVelocityEstimator:
    """Estimate control velocity from altitude, treating reported vario as diagnostic only."""

    def __init__(self, time_constant_s: float = 0.25) -> None:
        if time_constant_s < 0.0:
            raise ValueError("time_constant_s must not be negative")
        self._time_constant_s = time_constant_s
        self._previous_timestamp_s: float | None = None
        self._previous_altitude_m: float | None = None
        self._filtered_velocity_mps = 0.0

    def update(self, sample: AltitudeSample) -> tuple[float, float]:
        previous_timestamp_s = self._previous_timestamp_s
        previous_altitude_m = self._previous_altitude_m
        self._previous_timestamp_s = sample.timestamp_s
        self._previous_altitude_m = sample.altitude_m
        if previous_timestamp_s is None or previous_altitude_m is None:
            return self._filtered_velocity_mps, 0.0
        dt_s = sample.timestamp_s - previous_timestamp_s
        if dt_s <= 0.0:
            return self._filtered_velocity_mps, 0.0
        derived_velocity_mps = (sample.altitude_m - previous_altitude_m) / dt_s
        alpha = (
            1.0
            if self._time_constant_s == 0.0
            else dt_s / (self._time_constant_s + dt_s)
        )
        self._filtered_velocity_mps += alpha * (
            derived_velocity_mps - self._filtered_velocity_mps
        )
        return self._filtered_velocity_mps, dt_s
