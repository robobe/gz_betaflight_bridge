from __future__ import annotations

from msp_core.client import MspClient
from msp_core.protocol import MspProtocolError
from msp_core.rc import RcSender
from msp_core.telemetry import AltitudeTelemetry, StatusTelemetry

from .cli import parse_args
from .controller import HoverController, MissionFailure
from .flight_log import HoverFlightLog


def main() -> int:
    config = parse_args()
    print(f"Connecting to Betaflight MSP at {config.host}:{config.port}", flush=True)
    try:
        with HoverFlightLog(
            config.log_directory,
            config.hover,
            csv_flush_period_s=config.csv_flush_period_s,
            oscillation_deadband_m=config.oscillation_deadband_m,
        ) as recorder:
            controller = HoverController(
                config=config.hover,
                telemetry=AltitudeTelemetry(),
                status=StatusTelemetry(),
                rc_sender=RcSender(),
                recorder=recorder,
            )
            with MspClient(config.host, config.port, timeout_s=config.timeout_s) as client:
                controller.run(client)
    except KeyboardInterrupt:
        print("hover mission interrupted", flush=True)
        return 130
    except (OSError, TimeoutError, ValueError, MspProtocolError, MissionFailure) as exc:
        print(f"hover mission: FAIL: {exc}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
