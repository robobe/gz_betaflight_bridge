import importlib.util
import struct
import unittest
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    "check_mavlink_rangefinder", Path(__file__).parents[1] / "scripts/tools/check_mavlink_rangefinder.py"
)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class MavlinkRangefinderCheckerTest(unittest.TestCase):
    def test_parses_valid_mavlink2_distance_sensor(self) -> None:
        payload = struct.pack("<IHHHBBBB", 10, 20, 1200, 123, 0, 0, 0, 255)
        header = bytes([len(payload), 0, 0, 7, 1, 158]) + (132).to_bytes(3, "little")
        checksum = checker.crc_x25(header + payload + bytes([85]))
        packet = b"\xfd" + header + payload + struct.pack("<H", checksum)

        message_id, system_id, component_id, decoded = checker.parse_packet(packet)

        self.assertEqual((message_id, system_id, component_id), (132, 1, 158))
        self.assertIn("123cm", checker.describe_range(message_id, decoded, 0))


if __name__ == "__main__":
    unittest.main()
