from __future__ import annotations

import socket
import time

from .msp_protocol import MspProtocolError, build_request, try_parse_response


class MspClient:
    def __init__(self, host: str, port: int, timeout_s: float = 1.0) -> None:
        self._host = host
        self._port = port
        self._timeout_s = timeout_s
        self._socket: socket.socket | None = None
        self._buffer = bytearray()

    def connect(self) -> None:
        self.close()
        sock = socket.create_connection((self._host, self._port), timeout=self._timeout_s)
        sock.settimeout(0.05)
        self._socket = sock

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def send(self, command: int, payload: bytes = b"") -> None:
        self._require_socket().sendall(build_request(command, payload))

    def request(self, command: int, payload: bytes = b"", timeout_s: float | None = None) -> bytes:
        self.send(command, payload)
        deadline = time.monotonic() + (self._timeout_s if timeout_s is None else timeout_s)

        while time.monotonic() < deadline:
            response = try_parse_response(self._buffer)
            if response is not None:
                if response.command != command:
                    continue
                if response.is_error:
                    raise MspProtocolError(f"MSP command {command} returned an error response")
                return response.payload

            remaining = max(0.01, deadline - time.monotonic())
            self._require_socket().settimeout(min(0.05, remaining))
            try:
                chunk = self._require_socket().recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("MSP TCP connection closed")
            self._buffer.extend(chunk)

        raise TimeoutError(f"Timed out waiting for MSP command {command}")

    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise RuntimeError("MSP client is not connected")
        return self._socket

    def __enter__(self) -> MspClient:
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        self.close()

