from __future__ import annotations

import csv
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, TextIO


class CsvRecorder:
    """Buffered CSV output reusable by any mission or controller."""

    def __init__(
        self,
        path: Path,
        fieldnames: Sequence[str],
        *,
        flush_period_s: float = 1.0,
    ) -> None:
        if flush_period_s <= 0.0:
            raise ValueError("flush_period_s must be positive")
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file: TextIO = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=fieldnames, extrasaction="raise")
        self._writer.writeheader()
        self._flush_period_s = flush_period_s
        self._last_flush_s = time.monotonic()

    def write(self, row: Mapping[str, Any]) -> None:
        self._writer.writerow(row)
        now_s = time.monotonic()
        if now_s - self._last_flush_s >= self._flush_period_s:
            self._file.flush()
            self._last_flush_s = now_s

    def close(self) -> None:
        if self._file.closed:
            return
        self._file.flush()
        self._file.close()

    def __enter__(self) -> CsvRecorder:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        self.close()
