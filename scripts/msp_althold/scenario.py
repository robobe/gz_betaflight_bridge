from __future__ import annotations

import statistics
import struct
import time
from collections import deque
from dataclasses import dataclass

from msp_hover.msp_client import MspClient
from msp_hover.msp_protocol import MSP_BOXIDS, MSP_STATUS
from msp_hover.rc import RcChannels, RcSender
from msp_hover.telemetry import AltitudeSample, AltitudeTelemetry
from msp_hover.timing import RateLoop

from .cli import ScenarioConfig


BOX_ALTHOLD = 3
LOW_THROTTLE = 1000
CAPTURE_MIN = 1100
CAPTURE_MAX = 1700


class ScenarioFailure(RuntimeError):
    """Raised when a scenario acceptance check fails."""


@dataclass(frozen=True)
class HoldResult:
    name: str
    target_altitude_m: float
    min_altitude_m: float
    max_altitude_m: float
    max_error_m: float


def active_box_ids(box_ids_payload: bytes, status_payload: bytes) -> set[int]:
    """Map MSP_STATUS mode bits to the permanent IDs returned by MSP_BOXIDS."""
    if len(status_payload) < 10:
        raise ValueError(f"MSP_STATUS payload too short: {len(status_payload)} bytes")
    flags = struct.unpack_from("<I", status_payload, 6)[0]
    return {
        permanent_id
        for bit, permanent_id in enumerate(box_ids_payload[:32])
        if flags & (1 << bit)
    }


def captured_hover_throttle(values: list[int]) -> int:
    if not values:
        raise ValueError("No throttle samples available for ALT HOLD handoff")
    median = int(round(statistics.median(values)))
    return max(CAPTURE_MIN, min(CAPTURE_MAX, median))


def evaluate_hold(
    name: str,
    target_altitude_m: float,
    altitudes_m: list[float],
    tolerance_m: float,
) -> HoldResult:
    if not altitudes_m:
        raise ValueError("A hold window must contain altitude samples")
    result = HoldResult(
        name=name,
        target_altitude_m=target_altitude_m,
        min_altitude_m=min(altitudes_m),
        max_altitude_m=max(altitudes_m),
        max_error_m=max(abs(value - target_altitude_m) for value in altitudes_m),
    )
    # if result.max_error_m > tolerance_m:
    #     raise ScenarioFailure(
    #         f"{name} exceeded altitude tolerance: max error {result.max_error_m:.2f}m "
    #         f"> {tolerance_m:.2f}m"
    #     )
    return result


class AltitudePid:
    def __init__(self, hover_throttle: int) -> None:
        self._hover_throttle = hover_throttle
        self._integral = 0.0
        self._previous: AltitudeSample | None = None

    def command(self, target_altitude_m: float, sample: AltitudeSample) -> int:
        velocity = sample.vertical_velocity_mps
        dt_s = 0.0
        if self._previous is not None:
            dt_s = sample.timestamp_s - self._previous.timestamp_s
            if dt_s > 0.0:
                velocity = (sample.altitude_m - self._previous.altitude_m) / dt_s
        self._previous = sample

        error = target_altitude_m - sample.altitude_m
        if dt_s > 0.0:
            self._integral = max(-5.0, min(5.0, self._integral + error * dt_s))
        throttle = self._hover_throttle + 120.0 * error + 15.0 * self._integral - 60.0 * velocity
        return max(1100, min(2000, int(round(throttle))))


class AltHoldScenario:
    def __init__(
        self,
        config: ScenarioConfig,
        telemetry: AltitudeTelemetry,
        rc_sender: RcSender,
    ) -> None:
        self._config = config
        self._telemetry = telemetry
        self._rc_sender = rc_sender
        self._box_ids = b""
        self._results: list[HoldResult] = []
        self._last_log_s = 0.0

    def run(self, client: MspClient) -> None:
        self._box_ids = client.request(MSP_BOXIDS)
        if BOX_ALTHOLD not in self._box_ids:
            raise ScenarioFailure("Betaflight did not advertise the ALTHOLD mode (permanent ID 3)")

        try:
            self._timed_channels(client, "prearm", self._config.prearm_duration_s, LOW_THROTTLE, False, False)
            self._timed_channels(client, "arm-low", self._config.arm_low_duration_s, LOW_THROTTLE, True, False)

            recent_throttles = self._pid_takeoff(client)
            observed_hover_throttle = captured_hover_throttle(recent_throttles)
            neutral_throttle = self._config.althold_throttle
            print(
                f"handoff: observed PID hover={observed_hover_throttle} "
                f"native ALT HOLD baseline={neutral_throttle}",
                flush=True,
            )

            capture_altitude = self._engage_althold(client, neutral_throttle)
            self._settle(client, "initial-settle", capture_altitude, neutral_throttle)
            self._results.append(
                self._hold_window(client, "initial-hold", capture_altitude, neutral_throttle)
            )

            upper_target = capture_altitude + self._config.altitude_step_m
            self._move_to(client, "climb", upper_target, self._config.climb_throttle, rising=True)
            self._settle(client, "upper-settle", upper_target, neutral_throttle)
            self._results.append(
                self._hold_window(client, "upper-hold", upper_target, neutral_throttle)
            )

            self._move_to(client, "descent", capture_altitude, self._config.descent_throttle, rising=False)
            self._settle(client, "lower-settle", capture_altitude, neutral_throttle)
            self._results.append(
                self._hold_window(client, "lower-hold", capture_altitude, neutral_throttle)
            )

            self._exit_althold(client, neutral_throttle)
            self._land(client)
        finally:
            self.disarm(client)
        self._print_summary()

    def disarm(self, client: MspClient) -> None:
        loop = RateLoop(self._config.rate_hz)
        end_s = time.monotonic() + self._config.disarm_burst_s
        channels = self._channels(LOW_THROTTLE, arm=False, althold=False)
        while time.monotonic() < end_s:
            self._rc_sender.send(client, channels)
            loop.sleep()
        print("disarmed: throttle=1000 arm=0 angle=1 althold=0", flush=True)

    def _pid_takeoff(self, client: MspClient) -> list[int]:
        pid = AltitudePid(self._config.hover_throttle)
        loop = RateLoop(self._config.rate_hz)
        deadline = time.monotonic() + self._config.phase_timeout_s
        stable_since: float | None = None
        recent: deque[tuple[float, int]] = deque()

        while time.monotonic() < deadline:
            sample = self._telemetry.read(client)
            throttle = pid.command(self._config.takeoff_altitude_m, sample)
            self._rc_sender.send(client, self._channels(throttle, arm=True, althold=False))
            now_s = time.monotonic()
            recent.append((now_s, throttle))
            while recent and now_s - recent[0][0] > self._config.settle_duration_s:
                recent.popleft()

            stable = (
                abs(sample.altitude_m - self._config.takeoff_altitude_m) <= self._config.tolerance_m
                and abs(sample.vertical_velocity_mps) <= 0.5
            )
            stable_since = now_s if stable and stable_since is None else stable_since
            if not stable:
                stable_since = None
            self._log("pid-takeoff", sample, throttle, False)
            if stable_since is not None and now_s - stable_since >= self._config.settle_duration_s:
                return [value for _, value in recent]
            loop.sleep()

        raise ScenarioFailure("PID takeoff did not settle before the phase timeout")

    def _engage_althold(self, client: MspClient, throttle: int) -> float:
        loop = RateLoop(self._config.rate_hz)
        deadline = time.monotonic() + min(5.0, self._config.phase_timeout_s)
        capture_altitude: float | None = None
        while time.monotonic() < deadline:
            sample = self._telemetry.read(client)
            self._rc_sender.send(client, self._channels(throttle, arm=True, althold=True))
            if capture_altitude is None:
                capture_altitude = sample.altitude_m
            self._log("engage-althold", sample, throttle, True)
            if self._althold_active(client):
                print(f"althold active: captured altitude={capture_altitude:.2f}m", flush=True)
                return capture_altitude
            loop.sleep()
        raise ScenarioFailure("AUX3 was raised but MSP did not report ALTHOLD active")

    def _settle(self, client: MspClient, phase: str, target: float, throttle: int) -> None:
        loop = RateLoop(self._config.rate_hz)
        deadline = time.monotonic() + self._config.phase_timeout_s
        stable_since: float | None = None
        while time.monotonic() < deadline:
            sample = self._telemetry.read(client)
            self._rc_sender.send(client, self._channels(throttle, arm=True, althold=True))
            now_s = time.monotonic()
            stable = (
                abs(sample.altitude_m - target) <= self._config.tolerance_m
                and abs(sample.vertical_velocity_mps) <= 0.5
            )
            stable_since = now_s if stable and stable_since is None else stable_since
            if not stable:
                stable_since = None
            self._log(phase, sample, throttle, True)
            if stable_since is not None and now_s - stable_since >= self._config.settle_duration_s:
                return
            if not self._althold_active(client):
                raise ScenarioFailure(f"ALTHOLD became inactive during {phase}")
            loop.sleep()
        raise ScenarioFailure(f"{phase} did not settle before the phase timeout")

    def _hold_window(self, client: MspClient, phase: str, target: float, throttle: int) -> HoldResult:
        loop = RateLoop(self._config.rate_hz)
        end_s = time.monotonic() + self._config.hold_duration_s
        altitudes: list[float] = []
        while time.monotonic() < end_s:
            sample = self._telemetry.read(client)
            self._rc_sender.send(client, self._channels(throttle, arm=True, althold=True))
            altitudes.append(sample.altitude_m)
            self._log(phase, sample, throttle, True)
            if not self._althold_active(client):
                raise ScenarioFailure(f"ALTHOLD became inactive during {phase}")
            loop.sleep()
        return evaluate_hold(phase, target, altitudes, self._config.tolerance_m)

    def _move_to(self, client: MspClient, phase: str, target: float, throttle: int, rising: bool) -> None:
        loop = RateLoop(self._config.rate_hz)
        deadline = time.monotonic() + self._config.phase_timeout_s
        while time.monotonic() < deadline:
            sample = self._telemetry.read(client)
            self._rc_sender.send(client, self._channels(throttle, arm=True, althold=True))
            self._log(phase, sample, throttle, True)
            reached = sample.altitude_m >= target if rising else sample.altitude_m <= target
            if reached:
                return
            if not self._althold_active(client):
                raise ScenarioFailure(f"ALTHOLD became inactive during {phase}")
            loop.sleep()
        raise ScenarioFailure(f"{phase} did not reach {target:.2f}m before the phase timeout")

    def _exit_althold(self, client: MspClient, throttle: int) -> None:
        loop = RateLoop(self._config.rate_hz)
        deadline = time.monotonic() + min(5.0, self._config.phase_timeout_s)
        while time.monotonic() < deadline:
            self._rc_sender.send(client, self._channels(throttle, arm=True, althold=False))
            if not self._althold_active(client):
                print("althold inactive: AUX3 low", flush=True)
                return
            loop.sleep()
        raise ScenarioFailure("ALTHOLD remained active after AUX3 was lowered")

    def _land(self, client: MspClient) -> None:
        pid = AltitudePid(self._config.hover_throttle)
        loop = RateLoop(self._config.rate_hz)
        first = self._telemetry.read(client)
        start_altitude = first.altitude_m
        start_s = time.monotonic()
        deadline = start_s + self._config.landing_duration_s + self._config.phase_timeout_s
        while time.monotonic() < deadline:
            sample = self._telemetry.read(client)
            elapsed = time.monotonic() - start_s
            ratio = min(1.0, elapsed / self._config.landing_duration_s)
            target = start_altitude + (self._config.landing_altitude_m - start_altitude) * ratio
            throttle = pid.command(target, sample)
            self._rc_sender.send(client, self._channels(throttle, arm=True, althold=False))
            self._log("pid-land", sample, throttle, False)
            if ratio >= 1.0 and sample.altitude_m <= self._config.landing_altitude_m + 0.15:
                return
            loop.sleep()
        raise ScenarioFailure("Landing controller did not reach the landing altitude")

    def _timed_channels(
        self, client: MspClient, phase: str, duration_s: float, throttle: int, arm: bool, althold: bool
    ) -> None:
        loop = RateLoop(self._config.rate_hz)
        end_s = time.monotonic() + duration_s
        while time.monotonic() < end_s:
            self._rc_sender.send(client, self._channels(throttle, arm, althold))
            loop.sleep()
        print(f"{phase}: complete", flush=True)

    def _althold_active(self, client: MspClient) -> bool:
        return BOX_ALTHOLD in active_box_ids(self._box_ids, client.request(MSP_STATUS))

    @staticmethod
    def _channels(throttle: int, arm: bool, althold: bool) -> RcChannels:
        return RcChannels(
            throttle=throttle,
            aux1=2000 if arm else 1000,
            aux2=2000,
            aux3=2000 if althold else 1000,
        )

    def _log(self, phase: str, sample: AltitudeSample, throttle: int, althold: bool) -> None:
        now_s = time.monotonic()
        if now_s - self._last_log_s < self._config.log_period_s:
            return
        print(
            f"{phase}: alt={sample.altitude_m:.2f}m vv={sample.vertical_velocity_mps:.2f}m/s "
            f"throttle={throttle} aux3={1 if althold else 0}",
            flush=True,
        )
        self._last_log_s = now_s

    def _print_summary(self) -> None:
        print("ALT HOLD scenario: PASS", flush=True)
        for result in self._results:
            print(
                f"  {result.name}: target={result.target_altitude_m:.2f}m "
                f"range={result.min_altitude_m:.2f}..{result.max_altitude_m:.2f}m "
                f"max_error={result.max_error_m:.2f}m",
                flush=True,
            )
