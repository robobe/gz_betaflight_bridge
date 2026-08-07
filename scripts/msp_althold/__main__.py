from __future__ import annotations

from msp_core.client import MspClient
from msp_core.protocol import MspProtocolError
from msp_core.rc import RcSender
from msp_core.telemetry import AltitudeTelemetry

from .cli import parse_args
from .scenario import AltHoldScenario, ScenarioFailure


def main() -> int:
    config = parse_args()
    scenario = AltHoldScenario(config, AltitudeTelemetry(), RcSender())
    print(f"Connecting to Betaflight MSP at {config.host}:{config.port}", flush=True)
    try:
        with MspClient(config.host, config.port, timeout_s=config.timeout_s) as client:
            scenario.run(client)
    except KeyboardInterrupt:
        print("ALT HOLD scenario interrupted", flush=True)
        return 130
    except (OSError, TimeoutError, ValueError, MspProtocolError, ScenarioFailure) as exc:
        print(f"ALT HOLD scenario: FAIL: {exc}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
