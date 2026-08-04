from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from msp_core.telemetry import active_box_ids, decode_altitude, decode_attitude  # noqa: E402


class MspTelemetryTest(unittest.TestCase):
    def test_decodes_altitude_and_vario(self) -> None:
        sample = decode_altitude(struct.pack("<ih", 312, -45), timestamp_s=7.0)

        self.assertAlmostEqual(sample.altitude_m, 3.12)
        self.assertAlmostEqual(sample.vertical_velocity_mps, -0.45)
        self.assertEqual(sample.timestamp_s, 7.0)

    def test_decodes_attitude_units(self) -> None:
        sample = decode_attitude(struct.pack("<hhh", 123, -47, 181), timestamp_s=8.0)

        self.assertAlmostEqual(sample.roll_deg, 12.3)
        self.assertAlmostEqual(sample.pitch_deg, -4.7)
        self.assertEqual(sample.yaw_deg, 181.0)

    def test_maps_status_flags_to_permanent_box_ids(self) -> None:
        status = bytearray(10)
        struct.pack_into("<I", status, 6, (1 << 0) | (1 << 2))

        self.assertEqual(active_box_ids(bytes([0, 1, 13]), bytes(status)), {0, 13})

    def test_rejects_short_payloads(self) -> None:
        with self.assertRaisesRegex(ValueError, "MSP_ALTITUDE"):
            decode_altitude(b"\x00" * 5, 0.0)
        with self.assertRaisesRegex(ValueError, "MSP_ATTITUDE"):
            decode_attitude(b"\x00" * 5, 0.0)
        with self.assertRaisesRegex(ValueError, "MSP_STATUS"):
            active_box_ids(bytes([0]), b"\x00" * 9)


if __name__ == "__main__":
    unittest.main()
