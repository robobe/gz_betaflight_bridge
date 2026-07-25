from __future__ import annotations

from msp_hover.msp_client import MspClient
from msp_hover.rc import RcSender
from msp_hover.telemetry import AltitudeTelemetry

from .cli import parse_args
from .mission import SquareMissionController
from .pose import GazeboPoseSource


def main() -> int:
    config = parse_args()
    controller = SquareMissionController(
        config=config.mission,
        pose_source=GazeboPoseSource(config.pose_topic, model_name=config.model_name),
        telemetry=AltitudeTelemetry(),
        altitude_controller=config.altitude,
        position_controller=config.position,
        rc_sender=RcSender(),
    )

    print(f"Connecting to Betaflight MSP at {config.host}:{config.port}", flush=True)
    print(f"Subscribing to Gazebo pose topic {config.pose_topic} for model {config.model_name}", flush=True)
    with MspClient(config.host, config.port, timeout_s=config.timeout_s) as client:
        controller.run(client)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
