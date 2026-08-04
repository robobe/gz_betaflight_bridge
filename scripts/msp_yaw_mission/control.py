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
    def __init__(self) -> None:
        self._previous_timestamp_s: float | None = None

    def update(self, sample: AltitudeSample) -> tuple[float, float]:
        previous_timestamp_s = self._previous_timestamp_s
        self._previous_timestamp_s = sample.timestamp_s
        if previous_timestamp_s is None:
            return sample.vertical_velocity_mps, 0.0
        dt_s = sample.timestamp_s - previous_timestamp_s
        if dt_s <= 0.0:
            return sample.vertical_velocity_mps, 0.0
        return sample.vertical_velocity_mps, dt_s


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
