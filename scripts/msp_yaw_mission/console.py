from __future__ import annotations

import os
import sys
from enum import Enum


RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"


def colors_enabled() -> bool:
    return "NO_COLOR" not in os.environ and sys.stdout.isatty()


def colorize(
    message: str,
    color: str,
    *,
    bold: bool = False,
    enabled: bool | None = None,
) -> str:
    use_color = colors_enabled() if enabled is None else enabled
    if not use_color:
        return message
    prefix = f"{BOLD if bold else ''}{color}"
    return f"{prefix}{message}{RESET}"


def phase_color(phase: Enum) -> str:
    return {
        "prearm": BLUE,
        "arming": YELLOW,
        "liftoff": GREEN,
        "takeoff": GREEN,
        "settle": CYAN,
        "yaw_ccw_180": MAGENTA,
        "yaw_cw_home": BLUE,
        "descend": CYAN,
        "abort_descend": RED,
    }.get(str(phase.value), CYAN)


def transition_message(old: Enum, new: Enum, *, enabled: bool | None = None) -> str:
    message = f"mission: {old.value} -> {new.value}"
    return colorize(message, phase_color(new), bold=True, enabled=enabled)
