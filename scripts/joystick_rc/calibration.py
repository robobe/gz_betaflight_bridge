from __future__ import annotations

import time
from collections.abc import Callable

from .config import AxisBinding, ButtonBinding, JoystickRcConfig
from .device import JoystickDevice


class JoystickCalibrator:
    def __init__(
        self,
        device: JoystickDevice,
        input_func: Callable[[str], str] = input,
        print_func: Callable[..., None] = print,
    ) -> None:
        self._device = device
        self._input = input_func
        self._print = print_func

    def calibrate(self) -> JoystickRcConfig:
        self._print("Joystick RC calibration")
        self._print("Keep all sticks centered, then press Enter.")
        self._input("")
        centers = self._sample_centers(duration_s=1.0)

        roll = self._capture_axis("Move ROLL stick full RIGHT, then release it")
        pitch = self._capture_axis("Move PITCH stick full FORWARD, then release it")
        throttle = self._capture_axis("Move THROTTLE stick full HIGH, then release it")
        yaw = self._capture_axis("Move YAW stick full RIGHT, then release it")
        arm = self._capture_button("Press the ARM button or switch")
        arm_toggle = self._confirm("Use ARM as press-on / press-off toggle? [y/N] ")
        angle = self._capture_button("Press the ANGLE mode button or switch")
        angle_toggle = self._confirm("Use ANGLE as press-on / press-off toggle? [y/N] ")

        return JoystickRcConfig(
            roll=self._axis_binding(roll, positive_direction=True, centers=centers),
            pitch=self._axis_binding(pitch, positive_direction=False, centers=centers),
            throttle=self._axis_binding(throttle, positive_direction=True, centers=centers),
            yaw=self._axis_binding(yaw, positive_direction=True, centers=centers),
            arm=ButtonBinding(button=arm, toggle=arm_toggle),
            angle=ButtonBinding(button=angle, toggle=angle_toggle),
        )

    def _sample_centers(self, duration_s: float) -> dict[int, int]:
        end_s = time.monotonic() + duration_s
        latest: dict[int, int] = {}
        while time.monotonic() < end_s:
            event = self._device.read_event(timeout_s=0.05)
            if event and event.is_axis:
                latest[event.number] = event.value
        return latest

    def _capture_axis(self, prompt: str) -> tuple[int, int]:
        self._print("")
        self._print(prompt)
        self._print("Waiting for the largest axis movement...")
        end_s = time.monotonic() + 5.0
        best_axis = -1
        best_value = 0

        while time.monotonic() < end_s:
            event = self._device.read_event(timeout_s=0.1)
            if event is None or not event.is_axis or event.is_initial:
                continue
            if abs(event.value) > abs(best_value):
                best_axis = event.number
                best_value = event.value

        if best_axis < 0:
            raise TimeoutError(f"No axis movement detected for: {prompt}")

        self._print(f"Detected axis {best_axis} value={best_value}")
        return best_axis, best_value

    def _capture_button(self, prompt: str) -> int:
        self._print("")
        self._print(prompt)
        while True:
            event = self._device.read_event(timeout_s=10.0)
            if event is None:
                raise TimeoutError(f"No button press detected for: {prompt}")
            if event.is_button and not event.is_initial and event.value:
                self._print(f"Detected button {event.number}")
                return event.number

    def _confirm(self, prompt: str) -> bool:
        return self._input(prompt).strip().lower() in ("y", "yes")

    @staticmethod
    def _axis_binding(
        captured: tuple[int, int],
        positive_direction: bool,
        centers: dict[int, int],
    ) -> AxisBinding:
        axis, value = captured
        movement_is_positive = value >= centers.get(axis, 0)
        invert = movement_is_positive != positive_direction
        return AxisBinding(axis=axis, invert=invert, center=centers.get(axis, 0))
