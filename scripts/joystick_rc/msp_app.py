from __future__ import annotations

import argparse
import time
from pathlib import Path

from msp_core.client import MspClient
from msp_core.rc import RcChannels, RcSender

from .app import DEFAULT_CONFIG
from .calibration import JoystickCalibrator
from .config import load_config, save_config
from .device import JoystickDevice
from .mapper import JoystickState, RcFrame, RcMapper


class JoystickMspApp:
    def __init__(
        self,
        device_path: str,
        config_path: Path,
        host: str,
        port: int,
        timeout_s: float,
        rate_hz: float,
        print_period_s: float,
        disarm_burst_s: float,
    ) -> None:
        if rate_hz <= 0.0:
            raise ValueError("--rate must be greater than zero")
        self._device_path = device_path
        self._config_path = config_path
        self._host = host
        self._port = port
        self._timeout_s = timeout_s
        self._rate_hz = rate_hz
        self._print_period_s = print_period_s
        self._disarm_burst_s = disarm_burst_s

    def calibrate(self) -> None:
        with JoystickDevice(self._device_path) as device:
            config = JoystickCalibrator(device).calibrate()
        self._config_path.parent.mkdir(parents=True, exist_ok=True)
        save_config(self._config_path, config)
        print(f"Saved joystick RC config: {self._config_path}", flush=True)

    def run(self) -> None:
        config = load_config(self._config_path)
        mapper = RcMapper(config)
        rc_sender = RcSender()
        state = JoystickState()
        previous_buttons: dict[int, int] = {}
        period_s = 1.0 / self._rate_hz
        start_s = time.monotonic()
        next_send_s = start_s
        next_print_s = start_s
        last_frame: RcFrame | None = None

        print(
            f"Sending joystick RC from {self._device_path} to msp://{self._host}:{self._port} "
            f"at {self._rate_hz:.1f} Hz",
            flush=True,
        )
        print("Keep throttle low before enabling ARM.", flush=True)

        with MspClient(self._host, self._port, timeout_s=self._timeout_s) as client:
            try:
                with JoystickDevice(self._device_path) as device:
                    while True:
                        self._read_pending_events(device, state, previous_buttons)

                        now_s = time.monotonic()
                        if now_s >= next_send_s:
                            frame = mapper.map_state(state)
                            rc_sender.send(client, self._to_msp_channels(frame))
                            last_frame = frame
                            next_send_s += period_s

                        if last_frame is not None and now_s >= next_print_s:
                            self._print_frame(last_frame)
                            next_print_s = now_s + self._print_period_s

                        sleep_s = max(0.001, min(0.02, next_send_s - time.monotonic()))
                        time.sleep(sleep_s)
            except KeyboardInterrupt:
                print("Stopping joystick MSP RC.", flush=True)
            finally:
                self._send_disarm_burst(client, rc_sender, last_frame)

    @staticmethod
    def _read_pending_events(
        device: JoystickDevice,
        state: JoystickState,
        previous_buttons: dict[int, int],
    ) -> None:
        event = device.read_event(timeout_s=0.0)
        while event is not None:
            if event.is_axis:
                state.axes[event.number] = event.value
            elif event.is_button:
                previous_value = previous_buttons.get(event.number, 0)
                if event.value and not previous_value:
                    state.toggles[event.number] = not state.toggles.get(event.number, False)
                previous_buttons[event.number] = event.value
                state.buttons[event.number] = event.value
            event = device.read_event(timeout_s=0.0)

    def _send_disarm_burst(
        self,
        client: MspClient,
        rc_sender: RcSender,
        last_frame: RcFrame | None,
    ) -> None:
        end_s = time.monotonic() + self._disarm_burst_s
        aux2 = last_frame.aux2 if last_frame is not None else 2000
        channels = RcChannels(throttle=1000, aux1=1000, aux2=aux2)
        period_s = 1.0 / self._rate_hz
        while time.monotonic() < end_s:
            rc_sender.send(client, channels)
            time.sleep(period_s)
        print("disarmed: throttle=1000 arm=0", flush=True)

    @staticmethod
    def _to_msp_channels(frame: RcFrame) -> RcChannels:
        return RcChannels(
            roll=frame.roll,
            pitch=frame.pitch,
            throttle=frame.throttle,
            yaw=frame.yaw,
            aux1=frame.aux1,
            aux2=frame.aux2,
        )

    @staticmethod
    def _print_frame(frame: RcFrame) -> None:
        print(
            "rc: "
            f"roll={frame.roll} pitch={frame.pitch} throttle={frame.throttle} yaw={frame.yaw} "
            f"arm={1 if frame.aux1 > 1500 else 0} angle={1 if frame.aux2 > 1500 else 0}",
            flush=True,
        )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send Linux joystick input as Betaflight MSP RC commands.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    for subparser in (
        subparsers.add_parser("calibrate", help="Interactively bind joystick axes and buttons"),
        subparsers.add_parser("run", help="Send joystick RC commands to Betaflight MSP TCP"),
    ):
        subparser.add_argument("--device", default="/dev/input/js0")
        subparser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)

    run_parser = subparsers.choices["run"]
    run_parser.add_argument("--host", default="127.0.0.1")
    run_parser.add_argument("--port", type=int, default=5761)
    run_parser.add_argument("--timeout", type=float, default=1.0)
    run_parser.add_argument("--rate", type=float, default=50.0)
    run_parser.add_argument("--print-period", type=float, default=0.5)
    run_parser.add_argument("--disarm-burst", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    app = JoystickMspApp(
        device_path=args.device,
        config_path=args.config,
        host=getattr(args, "host", "127.0.0.1"),
        port=getattr(args, "port", 5761),
        timeout_s=getattr(args, "timeout", 1.0),
        rate_hz=getattr(args, "rate", 50.0),
        print_period_s=getattr(args, "print_period", 0.5),
        disarm_burst_s=getattr(args, "disarm_burst", 1.0),
    )

    if args.command == "calibrate":
        app.calibrate()
    elif args.command == "run":
        app.run()
    else:
        raise RuntimeError(f"Unknown command: {args.command}")
