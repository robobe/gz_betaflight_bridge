import importlib.util
import struct
import unittest
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    "check_rangefinder_msp", Path(__file__).parents[1] / "scripts/tools/check_rangefinder_msp.py"
)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class RangefinderMspCheckerTest(unittest.TestCase):
    def test_decodes_rangefinder_from_sensor_config_byte_three(self) -> None:
        config = bytes([0, 1, 0, 2, 0])
        status = bytes(4) + struct.pack("<H", 1 << 4)

        self.assertEqual(checker.decode_responses(config, status, struct.pack("<i", 123)), (2, True, 123))


if __name__ == "__main__":
    unittest.main()
