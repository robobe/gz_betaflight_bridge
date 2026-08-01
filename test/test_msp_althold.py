from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from msp_althold.scenario import (  # noqa: E402
    AltHoldScenario,
    ScenarioFailure,
    active_box_ids,
    captured_hover_throttle,
    evaluate_hold,
)


class AltHoldScenarioTest(unittest.TestCase):
    def test_active_box_ids_maps_status_bits_to_permanent_ids(self) -> None:
        box_ids = bytes([0, 1, 3, 13])
        status = bytearray(10)
        struct.pack_into("<I", status, 6, (1 << 0) | (1 << 2))

        self.assertEqual(active_box_ids(box_ids, bytes(status)), {0, 3})

    def test_active_box_ids_rejects_short_status(self) -> None:
        with self.assertRaisesRegex(ValueError, "too short"):
            active_box_ids(bytes([3]), bytes(9))

    def test_handoff_throttle_uses_median_and_betaflight_capture_limits(self) -> None:
        self.assertEqual(captured_hover_throttle([1660, 1680, 1700]), 1680)
        self.assertEqual(captured_hover_throttle([1740, 1750, 1760]), 1700)
        self.assertEqual(captured_hover_throttle([900, 1000, 1050]), 1100)

    def test_hold_window_accepts_values_inside_tolerance(self) -> None:
        result = evaluate_hold("hold", 3.0, [2.6, 3.0, 3.4], 0.5)

        self.assertAlmostEqual(result.max_error_m, 0.4)
        self.assertEqual(result.min_altitude_m, 2.6)
        self.assertEqual(result.max_altitude_m, 3.4)

    def test_hold_window_rejects_tolerance_violation(self) -> None:
        with self.assertRaisesRegex(ScenarioFailure, "exceeded altitude tolerance"):
            evaluate_hold("hold", 3.0, [3.0, 3.51], 0.5)

    def test_channel_states_keep_althold_low_for_arm_and_disarm(self) -> None:
        arm = AltHoldScenario._channels(1000, arm=True, althold=False).values()
        hold = AltHoldScenario._channels(1700, arm=True, althold=True).values()
        disarm = AltHoldScenario._channels(1000, arm=False, althold=False).values()

        self.assertEqual(arm[4:7], [2000, 2000, 1000])
        self.assertEqual(hold[4:7], [2000, 2000, 2000])
        self.assertEqual(disarm[2], 1000)
        self.assertEqual(disarm[4:7], [1000, 2000, 1000])


if __name__ == "__main__":
    unittest.main()
