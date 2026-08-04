from __future__ import annotations

from msp_core.client import MspClient
from msp_core.protocol import MspProtocolError
from msp_core.rc import RcSender
from msp_core.telemetry import AltitudeTelemetry, AttitudeTelemetry, StatusTelemetry

from .cli import parse_args
from .console import GREEN, RED, YELLOW, colorize
from .mission import MissionFailure, YawMission


def main() -> int:
    app = parse_args()
    mission = YawMission(
        app.mission,
        AltitudeTelemetry(),
        AttitudeTelemetry(),
        StatusTelemetry(),
        RcSender(),
    )
    print(f"Connecting to Betaflight MSP at {app.host}:{app.port}", flush=True)
    try:
        with MspClient(app.host, app.port, timeout_s=app.timeout_s) as client:
            mission.run(client)
    except KeyboardInterrupt:
        print(colorize("MISSION STOPPED: interrupted; disarm burst sent", YELLOW, bold=True), flush=True)
        return 130
    except (ConnectionError, OSError, TimeoutError, ValueError, MspProtocolError, MissionFailure) as exc:
        print(colorize(f"MISSION FAILED: {exc}", RED, bold=True), flush=True)
        return 1
    print(
        colorize("MISSION PASS: yaw excursion completed and vehicle disarmed", GREEN, bold=True),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
