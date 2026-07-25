from __future__ import annotations

import socket
import struct

from .mapper import RcFrame


RC_PACKET_FORMAT = "<d16H"


class UdpRcSender:
    def __init__(self, ip: str = "127.0.0.1", port: int = 9004) -> None:
        self._address = (ip, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def close(self) -> None:
        self._socket.close()

    def send(self, timestamp_s: float, frame: RcFrame) -> None:
        packet = struct.pack(RC_PACKET_FORMAT, timestamp_s, *frame.channels())
        self._socket.sendto(packet, self._address)
