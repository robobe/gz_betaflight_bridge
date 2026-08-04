from __future__ import annotations

import struct
from dataclasses import dataclass

from .client import MspClient
from .protocol import MSP_SET_RAW_RC


CHANNEL_COUNT = 16


@dataclass(frozen=True)
class RcChannels:
    roll: int = 1500
    pitch: int = 1500
    throttle: int = 1000
    yaw: int = 1500
    aux1: int = 1000
    aux2: int = 2000
    aux3: int = 1500
    aux4: int = 1500

    def values(self) -> list[int]:
        values = [
            self.roll,
            self.pitch,
            self.throttle,
            self.yaw,
            self.aux1,
            self.aux2,
            self.aux3,
            self.aux4,
        ]
        values.extend([1500] * (CHANNEL_COUNT - len(values)))
        return [clamp_channel(value) for value in values]


class RcSender:
    def send(self, client: MspClient, channels: RcChannels) -> None:
        payload = struct.pack("<16H", *channels.values())
        client.send(MSP_SET_RAW_RC, payload)


def clamp_channel(value: int) -> int:
    return max(800, min(2200, int(value)))
