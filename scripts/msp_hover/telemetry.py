from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from .msp_client import MspClient
from .msp_protocol import MSP_ALTITUDE


@dataclass(frozen=True)
class AltitudeSample:
    altitude_m: float
    vertical_velocity_mps: float
    timestamp_s: float


class AltitudeTelemetry:
    def read(self, client: MspClient) -> AltitudeSample:
        payload = client.request(MSP_ALTITUDE)
        if len(payload) < 6:
            raise ValueError(f"MSP_ALTITUDE payload too short: {len(payload)} bytes")

        altitude_cm, vario_cms = struct.unpack_from("<ih", payload)
        return AltitudeSample(
            altitude_m=altitude_cm / 100.0,
            vertical_velocity_mps=vario_cms / 100.0,
            timestamp_s=time.monotonic(),
        )

