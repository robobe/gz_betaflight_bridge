import importlib.util
import math
import struct
import unittest
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    "check_mavlink_odometry", Path(__file__).parents[1] / "scripts/tools/check_mavlink_odometry.py"
)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class MavlinkOdometryCheckerTest(unittest.TestCase):
    def test_parses_valid_mavlink2_odometry(self) -> None:
        payload = bytearray(233)
        struct.pack_into("<Q13f", payload, 0, 123456, 1, 2, -3, 1, 0, 0, 0, 4, -5, -6, .1, -.2, -.3)
        struct.pack_into("<f", payload, 60, math.nan)
        struct.pack_into("<f", payload, 144, math.nan)
        struct.pack_into("<BBBBb", payload, 228, 1, 12, 2, 1, 0)
        header = bytes([len(payload), 0, 0, 7, 1, 158]) + (331).to_bytes(3, "little")
        checksum = checker.crc_x25(header + payload + bytes([91]))
        packet = b"\xfd" + header + payload + struct.pack("<H", checksum)

        message_id, system_id, component_id, decoded = checker.parse_packet(packet)

        self.assertEqual((message_id, system_id, component_id), (331, 1, 158))
        self.assertIn("position=(1.0, 2.0, -3.0)", checker.describe_odometry(decoded))


if __name__ == "__main__":
    unittest.main()
