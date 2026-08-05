#!/usr/bin/env python3
"""GUI for commanding one Gazebo multicopter actuator at a time."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tkinter as tk
from tkinter import messagebox, ttk


DEFAULT_TOPIC = "/X3/gazebo/command/motor_speed"
PUBLISH_PERIOD_MS = 200
MOTOR_OPTIONS = (
    "0 - front-right (CCW)",
    "1 - rear-left (CCW)",
    "2 - front-left (CW)",
    "3 - rear-right (CW)",
)


def actuator_message(motor: int | None, velocity: float = 0.0) -> str:
    values = [0.0] * 4
    if motor is not None:
        if motor not in range(4):
            raise ValueError("motor must be between 0 and 3")
        values[motor] = velocity
    formatted = ", ".join(f"{value:g}" for value in values)
    return f"velocity:[{formatted}]"


class MotorVelocityGui:
    def __init__(self, root: tk.Tk, topic: str, max_velocity: float) -> None:
        self._root = root
        self._topic = topic
        self._max_velocity = max_velocity
        self._running = False
        self._active_motor = 0
        self._active_velocity = 0.0
        self._publish_after_id: str | None = None

        root.title("Gazebo Motor Test")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.close)

        frame = ttk.Frame(root, padding=14)
        frame.grid()

        ttk.Label(frame, text="SDF motor / actuator").grid(row=0, column=0, sticky="w")
        self._motor = ttk.Combobox(frame, values=MOTOR_OPTIONS, state="readonly", width=27)
        self._motor.current(0)
        self._motor.grid(row=1, column=0, columnspan=2, pady=(3, 10), sticky="ew")

        ttk.Label(frame, text="Velocity (rad/s)").grid(row=2, column=0, sticky="w")
        self._velocity = ttk.Entry(frame, width=12)
        self._velocity.insert(0, "200")
        self._velocity.grid(row=3, column=0, columnspan=2, pady=(3, 12), sticky="ew")

        self._start = ttk.Button(frame, text="Start", command=self.start)
        self._start.grid(row=4, column=0, padx=(0, 5), sticky="ew")
        self._stop = ttk.Button(frame, text="Stop", command=self.stop, state="disabled")
        self._stop.grid(row=4, column=1, padx=(5, 0), sticky="ew")

        self._status = tk.StringVar(value="Stopped: all motors are zero")
        ttk.Label(frame, textvariable=self._status).grid(
            row=5, column=0, columnspan=2, pady=(12, 0), sticky="w"
        )

    def _publish(self, message: str) -> None:
        completed = subprocess.run(
            [
                "gz",
                "topic",
                "-t",
                self._topic,
                "--msgtype",
                "gz.msgs.Actuators",
                "-p",
                message,
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip() or "unknown gz error"
            raise RuntimeError(detail)

    def start(self) -> None:
        try:
            velocity = float(self._velocity.get())
            if not 0.0 < velocity <= self._max_velocity:
                raise ValueError(f"velocity must be greater than 0 and at most {self._max_velocity:g}")
            motor = self._motor.current()
            self._publish(actuator_message(motor, velocity))
        except (ValueError, RuntimeError) as exc:
            messagebox.showerror("Motor command failed", str(exc), parent=self._root)
            return

        self._running = True
        self._active_motor = motor
        self._active_velocity = velocity
        self._motor.configure(state="disabled")
        self._velocity.configure(state="disabled")
        self._start.configure(state="disabled")
        self._stop.configure(state="normal")
        self._status.set(f"Running actuator {motor} at {velocity:g} rad/s")
        self._schedule_publish()

    def _schedule_publish(self) -> None:
        self._publish_after_id = self._root.after(PUBLISH_PERIOD_MS, self._publish_running)

    def _publish_running(self) -> None:
        self._publish_after_id = None
        if not self._running:
            return
        try:
            self._publish(actuator_message(self._active_motor, self._active_velocity))
        except RuntimeError as exc:
            self._set_stopped_state()
            messagebox.showerror("Motor command failed", str(exc), parent=self._root)
            return
        self._schedule_publish()

    def _set_stopped_state(self) -> None:
        self._running = False
        if self._publish_after_id is not None:
            self._root.after_cancel(self._publish_after_id)
            self._publish_after_id = None
        self._motor.configure(state="readonly")
        self._velocity.configure(state="normal")
        self._start.configure(state="normal")
        self._stop.configure(state="disabled")
        self._status.set("Stopped: all motors are zero")

    def stop(self, show_error: bool = True) -> None:
        self._running = False
        if self._publish_after_id is not None:
            self._root.after_cancel(self._publish_after_id)
            self._publish_after_id = None
        try:
            self._publish(actuator_message(None))
        except RuntimeError as exc:
            self._set_stopped_state()
            if show_error:
                messagebox.showerror("Stop command failed", str(exc), parent=self._root)
            return

        self._set_stopped_state()

    def close(self) -> None:
        if self._running:
            self.stop(show_error=False)
        else:
            # Send a stop command even if this GUI did not start the motor.
            try:
                self._publish(actuator_message(None))
            except RuntimeError:
                pass
        self._root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--max-velocity", type=float, default=1000.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.max_velocity <= 0.0:
        raise SystemExit("--max-velocity must be greater than zero")
    if shutil.which("gz") is None:
        raise SystemExit("gz command not found; install Gazebo and source its environment")

    root = tk.Tk()
    MotorVelocityGui(root, args.topic, args.max_velocity)
    root.mainloop()


if __name__ == "__main__":
    main()
