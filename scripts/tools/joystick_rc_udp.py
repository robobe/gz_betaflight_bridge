#!/usr/bin/env python3
"""Send Linux joystick input to Betaflight SITL UDP RC port 9004."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from joystick_rc.__main__ import main  # noqa: E402


if __name__ == "__main__":
    main()
