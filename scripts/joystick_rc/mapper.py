from __future__ import annotations

from dataclasses import dataclass, field

from .config import AxisBinding, ButtonBinding, JoystickRcConfig


AXIS_MAX = 32767
AXIS_MIN = -32768


@dataclass
class JoystickState:
    axes: dict[int, int] = field(default_factory=dict)
    buttons: dict[int, int] = field(default_factory=dict)
    toggles: dict[int, bool] = field(default_factory=dict)


@dataclass(frozen=True)
class RcFrame:
    roll: int
    pitch: int
    throttle: int
    yaw: int
    aux1: int
    aux2: int

    def channels(self) -> list[int]:
        values = [
            self.roll,
            self.pitch,
            self.throttle,
            self.yaw,
            self.aux1,
            self.aux2,
        ]
        values.extend([1500] * (16 - len(values)))
        return values


class RcMapper:
    def __init__(self, config: JoystickRcConfig) -> None:
        self._config = config

    def map_state(self, state: JoystickState) -> RcFrame:
        return RcFrame(
            roll=self._centered_axis_to_rc(state, self._config.roll),
            pitch=self._centered_axis_to_rc(state, self._config.pitch),
            throttle=self._throttle_axis_to_rc(state, self._config.throttle),
            yaw=self._centered_axis_to_rc(state, self._config.yaw),
            aux1=self._button_to_rc(state, self._config.arm),
            aux2=self._button_to_rc(state, self._config.angle),
        )

    def _centered_axis_to_rc(self, state: JoystickState, binding: AxisBinding) -> int:
        normalized = self._normalize_axis(state.axes.get(binding.axis, binding.center), binding)
        normalized = self._apply_deadzone_and_expo(normalized, self._config.deadzone, self._config.axis_expo)
        span = self._config.max_rc - self._config.mid_rc
        return self._clamp_rc(round(self._config.mid_rc + normalized * span))

    def _throttle_axis_to_rc(self, state: JoystickState, binding: AxisBinding) -> int:
        normalized = self._normalize_axis(state.axes.get(binding.axis, AXIS_MIN), binding)
        normalized = max(-1.0, min(1.0, normalized))
        ratio = (normalized + 1.0) / 2.0
        ratio = self._apply_positive_expo(ratio, self._config.throttle_expo)
        return self._clamp_rc(round(self._config.min_rc + ratio * (self._config.max_rc - self._config.min_rc)))

    def _normalize_axis(self, raw_value: int, binding: AxisBinding) -> float:
        centered = raw_value - binding.center
        if centered >= 0:
            denominator = max(1, AXIS_MAX - binding.center)
        else:
            denominator = max(1, binding.center - AXIS_MIN)
        normalized = centered / denominator
        if binding.invert:
            normalized *= -1.0
        return max(-1.0, min(1.0, normalized))

    @staticmethod
    def _apply_deadzone_and_expo(value: float, deadzone: float, expo: float) -> float:
        deadzone = max(0.0, min(0.95, deadzone))
        magnitude = abs(value)
        if magnitude <= deadzone:
            return 0.0
        scaled = (magnitude - deadzone) / (1.0 - deadzone)
        curved = RcMapper._apply_expo(scaled, expo)
        return curved if value >= 0.0 else -curved

    @staticmethod
    def _apply_expo(value: float, expo: float) -> float:
        expo = max(0.0, min(1.0, expo))
        return (1.0 - expo) * value + expo * value * value * value

    @staticmethod
    def _apply_positive_expo(value: float, expo: float) -> float:
        expo = max(0.0, min(1.0, expo))
        value = max(0.0, min(1.0, value))
        return (1.0 - expo) * value + expo * value * value * value

    def _button_to_rc(self, state: JoystickState, binding: ButtonBinding) -> int:
        if binding.toggle:
            return self._config.max_rc if state.toggles.get(binding.button, False) else self._config.min_rc
        return self._config.max_rc if state.buttons.get(binding.button, 0) else self._config.min_rc

    def _clamp_rc(self, value: int) -> int:
        return max(self._config.min_rc, min(self._config.max_rc, value))
