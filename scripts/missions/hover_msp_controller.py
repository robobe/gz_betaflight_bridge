#!/usr/bin/env python3
"""Run the MSP-based Betaflight SITL hover controller."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from msp_hover.__main__ import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
