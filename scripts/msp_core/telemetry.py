from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from .client import MspClient
from .protocol import MSP_ALTITUDE, MSP_ATTITUDE, MSP_BOXIDS, MSP_STATUS


ARM_PERMANENT_ID = 0
ANGLE_PERMANENT_ID = 1


@dataclass(frozen=True)
class AltitudeSample:
    altitude_m: float
    vertical_velocity_mps: float
    timestamp_s: float


@dataclass(frozen=True)
class AttitudeSample:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    timestamp_s: float


@dataclass(frozen=True)
class FlightStatus:
    armed: bool
    angle_mode: bool
    active_mode_ids: frozenset[int]
    timestamp_s: float


class AltitudeTelemetry:
    def read(self, client: MspClient) -> AltitudeSample:
        return decode_altitude(client.request(MSP_ALTITUDE), time.monotonic())


class AttitudeTelemetry:
    def read(self, client: MspClient) -> AttitudeSample:
        return decode_attitude(client.request(MSP_ATTITUDE), time.monotonic())


class StatusTelemetry:
    def __init__(self) -> None:
        self._box_ids: bytes | None = None

    def read(self, client: MspClient) -> FlightStatus:
        if self._box_ids is None:
            self._box_ids = client.request(MSP_BOXIDS)
        active = active_box_ids(self._box_ids, client.request(MSP_STATUS))
        return FlightStatus(
            armed=ARM_PERMANENT_ID in active,
            angle_mode=ANGLE_PERMANENT_ID in active,
            active_mode_ids=frozenset(active),
            timestamp_s=time.monotonic(),
        )


def decode_altitude(payload: bytes, timestamp_s: float) -> AltitudeSample:
    if len(payload) < 6:
        raise ValueError(f"MSP_ALTITUDE payload too short: {len(payload)} bytes")
    altitude_cm, vario_cms = struct.unpack_from("<ih", payload)
    return AltitudeSample(altitude_cm / 100.0, vario_cms / 100.0, timestamp_s)


def decode_attitude(payload: bytes, timestamp_s: float) -> AttitudeSample:
    if len(payload) < 6:
        raise ValueError(f"MSP_ATTITUDE payload too short: {len(payload)} bytes")
    roll_decideg, pitch_decideg, yaw_deg = struct.unpack_from("<hhh", payload)
    return AttitudeSample(roll_decideg / 10.0, pitch_decideg / 10.0, float(yaw_deg), timestamp_s)


def active_box_ids(box_ids: bytes, status_payload: bytes) -> set[int]:
    if len(status_payload) < 10:
        raise ValueError(f"MSP_STATUS payload too short: {len(status_payload)} bytes")
    active_flags = struct.unpack_from("<I", status_payload, 6)[0]
    return {permanent_id for index, permanent_id in enumerate(box_ids) if active_flags & (1 << index)}
