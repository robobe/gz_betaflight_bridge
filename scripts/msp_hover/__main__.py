from __future__ import annotations

from msp_core.client import MspClient
from msp_core.rc import RcSender
from msp_core.telemetry import AltitudeTelemetry

from .cli import parse_args
from .controller import HoverController


def main() -> int:
    config = parse_args()
    controller = HoverController(
        config=config.hover,
        telemetry=AltitudeTelemetry(),
        rc_sender=RcSender(),
    )

    print(f"Connecting to Betaflight MSP at {config.host}:{config.port}", flush=True)
    with MspClient(config.host, config.port, timeout_s=config.timeout_s) as client:
        controller.run(client)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
