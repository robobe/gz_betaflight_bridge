from __future__ import annotations

import time


class RateLoop:
    def __init__(self, rate_hz: float) -> None:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self._period_s = 1.0 / rate_hz
        self._next_time_s = time.monotonic()

    def sleep(self) -> None:
        self._next_time_s += self._period_s
        delay_s = self._next_time_s - time.monotonic()
        if delay_s > 0.0:
            time.sleep(delay_s)
        else:
            self._next_time_s = time.monotonic()

