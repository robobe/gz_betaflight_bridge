from __future__ import annotations

from msp_core.client import MspClient
from msp_core.protocol import MspProtocolError
from msp_core.rc import RcSender
from msp_core.telemetry import AltitudeTelemetry, AttitudeTelemetry, StatusTelemetry

from .cli import parse_args
from .controller import MissionFailure, YawMissionController
from .flight_log import YawFlightLog


def main() -> int:
    app = parse_args()
    print(f"Connecting to Betaflight MSP at {app.host}:{app.port}", flush=True)
    try:
        with YawFlightLog(
            app.log_directory,
            app.mission,
            csv_flush_period_s=app.csv_flush_period_s,
        ) as recorder:
            mission = YawMissionController(
                app.mission,
                AltitudeTelemetry(),
                AttitudeTelemetry(),
                StatusTelemetry(),
                RcSender(),
                recorder,
            )
            with MspClient(app.host, app.port, timeout_s=app.timeout_s) as client:
                mission.run(client)
    except KeyboardInterrupt:
        print("MISSION STOPPED: interrupted; disarm burst sent", flush=True)
        return 130
    except (ConnectionError, OSError, TimeoutError, ValueError, MspProtocolError, MissionFailure) as exc:
        print(f"MISSION FAILED: {exc}", flush=True)
        return 1
    print("MISSION PASS: CCW/CW yaw excursion completed and vehicle disarmed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
