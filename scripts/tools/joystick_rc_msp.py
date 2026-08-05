#!/usr/bin/env python3
"""Send Linux joystick input to Betaflight MSP RC."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from joystick_rc.msp_app import main  # noqa: E402


if __name__ == "__main__":
    main()
