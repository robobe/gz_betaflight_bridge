from __future__ import annotations

from dataclasses import dataclass


MSP_API_VERSION = 1
MSP_STATUS = 101
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_BOXIDS = 119
MSP_SET_RAW_RC = 200

REQUEST_HEADER = b"$M<"
RESPONSE_HEADER = b"$M>"
ERROR_HEADER = b"$M!"


class MspProtocolError(RuntimeError):
    """Raised when an MSP frame is malformed or reports an error."""


@dataclass(frozen=True)
class MspResponse:
    command: int
    payload: bytes
    is_error: bool = False


def checksum(size: int, command: int, payload: bytes) -> int:
    value = size ^ command
    for byte in payload:
        value ^= byte
    return value & 0xFF


def build_request(command: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 255:
        raise ValueError(f"MSP v1 command out of range: {command}")
    if len(payload) > 255:
        raise ValueError("MSP v1 payload must be 255 bytes or less")

    size = len(payload)
    return REQUEST_HEADER + bytes([size, command]) + payload + bytes([checksum(size, command, payload)])


def try_parse_response(buffer: bytearray) -> MspResponse | None:
    while len(buffer) >= 3:
        if buffer[:2] != b"$M":
            del buffer[0]
            continue
        if buffer[2:3] not in (b">", b"!"):
            del buffer[0]
            continue
        break

    if len(buffer) < 6:
        return None

    header = bytes(buffer[:3])
    size = buffer[3]
    frame_size = 3 + 1 + 1 + size + 1
    if len(buffer) < frame_size:
        return None

    command = buffer[4]
    payload = bytes(buffer[5 : 5 + size])
    expected = checksum(size, command, payload)
    actual = buffer[frame_size - 1]
    del buffer[:frame_size]

    if actual != expected:
        raise MspProtocolError(
            f"Bad MSP checksum for command {command}: expected 0x{expected:02x}, got 0x{actual:02x}"
        )

    return MspResponse(command=command, payload=payload, is_error=(header == ERROR_HEADER))
