import importlib.util
import socket
import unittest
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    "configure_rangefinder_cli", Path(__file__).parents[1] / "scripts/tools/configure_rangefinder_cli.py"
)
configurator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(configurator)


class FakeSocket:
    def __init__(self) -> None:
        self.responses = [b"Entering CLI Mode\r\n# ", b"Available: RANGEFINDER\r\n# ", b"# ", b"# ", b"# ", b"Rebooting"]
        self.sent = []

    def sendall(self, data: bytes) -> None:
        self.sent.append(data)

    def recv(self, _size: int) -> bytes:
        return self.responses.pop(0)


class ConfigureRangefinderCliTest(unittest.TestCase):
    def test_sends_complete_configuration(self) -> None:
        sock = FakeSocket()
        configurator.configure(sock)
        self.assertEqual(sock.sent, [b"#", b"feature list\r\n"] + [command.encode() + b"\r\n" for command in configurator.COMMANDS])
        self.assertIn(b"set rangefinder_hardware=TFMINI\r\n", sock.sent)

    def test_rejects_sitl_without_rangefinder_support(self) -> None:
        sock = FakeSocket()
        sock.responses[1] = b"Unavailable: RANGEFINDER TELEMETRY\r\n# "
        with self.assertRaisesRegex(RuntimeError, "rebuild SITL"):
            configurator.configure(sock)


if __name__ == "__main__":
    unittest.main()
