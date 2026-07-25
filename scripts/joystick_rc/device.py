from __future__ import annotations

import os
import select
import struct
from dataclasses import dataclass
from pathlib import Path


JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


@dataclass(frozen=True)
class JoystickEvent:
    timestamp_ms: int
    value: int
    event_type: int
    number: int

    @property
    def is_axis(self) -> bool:
        return bool(self.event_type & JS_EVENT_AXIS)

    @property
    def is_button(self) -> bool:
        return bool(self.event_type & JS_EVENT_BUTTON)

    @property
    def is_initial(self) -> bool:
        return bool(self.event_type & JS_EVENT_INIT)


class JoystickDevice:
    def __init__(self, path: str = "/dev/input/js0") -> None:
        self._path = Path(path)
        self._fd: int | None = None

    def __enter__(self) -> JoystickDevice:
        self.open()
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def open(self) -> None:
        if self._fd is not None:
            return
        self._fd = os.open(self._path, os.O_RDONLY | os.O_NONBLOCK)

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def read_event(self, timeout_s: float | None = None) -> JoystickEvent | None:
        if self._fd is None:
            raise RuntimeError("Joystick device is not open")

        readable, _, _ = select.select([self._fd], [], [], timeout_s)
        if not readable:
            return None

        try:
            data = os.read(self._fd, JS_EVENT_SIZE)
        except BlockingIOError:
            return None

        if len(data) != JS_EVENT_SIZE:
            return None

        timestamp_ms, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
        return JoystickEvent(
            timestamp_ms=timestamp_ms,
            value=value,
            event_type=event_type,
            number=number,
        )
