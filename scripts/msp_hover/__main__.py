from __future__ import annotations

from .cli import parse_args
from .controller import HoverController
from .msp_client import MspClient
from .rc import RcSender
from .telemetry import AltitudeTelemetry


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

