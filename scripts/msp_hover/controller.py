from __future__ import annotations

import time
from dataclasses import dataclass

from .msp_client import MspClient
from .rc import RcChannels, RcSender
from .telemetry import AltitudeSample, AltitudeTelemetry
from .timing import RateLoop


@dataclass(frozen=True)
class HoverConfig:
    target_altitude_m: float = 5.0
    rate_hz: float = 50.0
    duration_s: float = 0.0
    hover_throttle: int = 1600
    kp: float = 100.0
    kd: float = 90.0
    min_throttle: int = 1100
    max_throttle: int = 2000
    prearm_duration_s: float = 3.0
    arm_low_duration_s: float = 1.0
    log_period_s: float = 1.0
    low_throttle: int = 1000
    disarm_burst_s: float = 1.0
    angle_mode: bool = True


class HoverController:
    def __init__(
        self,
        config: HoverConfig,
        telemetry: AltitudeTelemetry,
        rc_sender: RcSender,
    ) -> None:
        self._config = config
        self._telemetry = telemetry
        self._rc_sender = rc_sender
        self._previous_sample: AltitudeSample | None = None

    def run(self, client: MspClient) -> None:
        loop = RateLoop(self._config.rate_hz)
        start_s = time.monotonic()
        last_log_s = 0.0

        try:
            while True:
                now_s = time.monotonic()
                elapsed_s = now_s - start_s
                if self._config.duration_s > 0.0 and elapsed_s >= self._config.duration_s:
                    break

                phase, channels, altitude_m, vertical_velocity_mps = self._step(client, elapsed_s)

                if now_s - last_log_s >= self._config.log_period_s:
                    print(
                        f"{phase}: alt={altitude_m:.2f}m vv={vertical_velocity_mps:.2f}m/s "
                        f"target={self._config.target_altitude_m:.2f}m throttle={channels.throttle}",
                        flush=True,
                    )
                    last_log_s = now_s

                loop.sleep()
        finally:
            self.disarm(client)

    def disarm(self, client: MspClient) -> None:
        loop = RateLoop(self._config.rate_hz)
        end_s = time.monotonic() + self._config.disarm_burst_s
        channels = self._channels(throttle=self._config.low_throttle, arm=False)
        while time.monotonic() < end_s:
            self._rc_sender.send(client, channels)
            loop.sleep()
        angle = 1 if self._config.angle_mode else 0
        print(f"disarmed: throttle=1000 arm=0 angle={angle}", flush=True)

    def _step(self, client: MspClient, elapsed_s: float) -> tuple[str, RcChannels, float, float]:
        sample = self._telemetry.read(client)
        vertical_velocity_mps = self._derive_vertical_velocity(sample)

        if elapsed_s < self._config.prearm_duration_s:
            phase = "prearm"
            channels = self._channels(throttle=self._config.low_throttle, arm=False)
        elif elapsed_s < self._config.prearm_duration_s + self._config.arm_low_duration_s:
            phase = "arm-low"
            channels = self._channels(throttle=self._config.low_throttle, arm=True)
        else:
            phase = "hover"
            throttle = self._hover_throttle(sample.altitude_m, vertical_velocity_mps)
            channels = self._channels(throttle=throttle, arm=True)

        self._rc_sender.send(client, channels)
        return phase, channels, sample.altitude_m, vertical_velocity_mps

    def _derive_vertical_velocity(self, sample: AltitudeSample) -> float:
        previous = self._previous_sample
        self._previous_sample = sample

        if previous is None:
            return sample.vertical_velocity_mps

        dt_s = sample.timestamp_s - previous.timestamp_s
        if dt_s <= 0.0:
            return sample.vertical_velocity_mps

        return (sample.altitude_m - previous.altitude_m) / dt_s

    def _hover_throttle(self, altitude_m: float, vertical_velocity_mps: float) -> int:
        error_m = self._config.target_altitude_m - altitude_m
        throttle = self._config.hover_throttle + self._config.kp * error_m - self._config.kd * vertical_velocity_mps
        return max(self._config.min_throttle, min(self._config.max_throttle, int(round(throttle))))

    def _channels(self, throttle: int, arm: bool) -> RcChannels:
        return RcChannels(
            throttle=throttle,
            aux1=2000 if arm else 1000,
            aux2=2000 if self._config.angle_mode else 1000,
        )
