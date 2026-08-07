#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from msp_hover.tuning import compare_trials  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare repeated MSP hover PID tuning trials.")
    parser.add_argument("--baseline", type=Path, nargs="+", required=True)
    parser.add_argument("--candidate", type=Path, nargs="+", required=True)
    args = parser.parse_args()
    try:
        comparison = compare_trials(args.baseline, args.candidate)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f"decision: {'KEEP' if comparison.accepted else 'REJECT'}")
    print(f"reason: {comparison.reason}")
    print(f"baseline median: {asdict(comparison.baseline)}")
    print(f"candidate median: {asdict(comparison.candidate)}")
    return 0 if comparison.accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
