#!/usr/bin/env python3
"""Configure Betaflight SITL UART2 for the bridge's TFmini rangefinder."""

import argparse
import socket
import sys
import re


COMMANDS = (
    "feature RANGEFINDER",
    "serial UART2 32768 115200 115200 0 115200",
    "set rangefinder_hardware=TFMINI",
    "save",
)


def read_response(sock: socket.socket, save: bool = False) -> str:
    data = bytearray()
    while True:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            raise TimeoutError("timed out waiting for Betaflight CLI") from None
        if not chunk:
            if save:
                break
            raise ConnectionError("Betaflight closed the CLI connection")
        data.extend(chunk)
        if (save and b"Rebooting" in data) or (not save and data.rstrip().endswith(b"#")):
            break
    response = data.decode("utf-8", "replace")
    if "ERROR" in response.upper() or "PARSE ERROR" in response.upper():
        raise RuntimeError(response.strip())
    return response


def configure(sock: socket.socket) -> None:
    sock.sendall(b"#")
    read_response(sock)
    sock.sendall(b"feature list\r\n")
    features = read_response(sock)
    if re.search(r"Unavailable:.*\bRANGEFINDER\b", features):
        raise RuntimeError("RANGEFINDER is unavailable; rebuild SITL with scripts/builders/build_betaflight_sitl.sh")
    for command in COMMANDS:
        print(f"> {command}")
        sock.sendall(command.encode() + b"\r\n")
        read_response(sock, save=(command == "save"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or args.timeout <= 0:
        parser.error("port must be 1..65535 and timeout must be positive")

    try:
        with socket.create_connection((args.host, args.port), args.timeout) as sock:
            sock.settimeout(args.timeout)
            configure(sock)
    except (OSError, RuntimeError, TimeoutError) as exc:
        print(f"Rangefinder configuration failed: {exc}", file=sys.stderr)
        return 1

    print("Rangefinder configured; Betaflight is rebooting.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
