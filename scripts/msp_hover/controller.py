from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from flight_control.altitude import AltitudePid, ThrottleSlewLimiter, VerticalVelocityEstimator
from msp_core.client import MspClient
from msp_core.rc import RcChannels, RcSender
from msp_core.telemetry import AltitudeTelemetry, FlightStatus, StatusTelemetry
from msp_core.timing import RateLoop


class MissionFailure(RuntimeError):
    """Raised after a safety or acceptance failure."""


class Phase(str, Enum):
    PREARM = "prearm"
    ARMING = "arming"
    LIFTOFF = "liftoff"
    TAKEOFF = "takeoff"
    SETTLE = "settle"
    SCORED_HOVER = "scored_hover"
    DESCEND = "descend"
    ABORT_DESCEND = "abort_descend"


@dataclass(frozen=True)
class HoverConfig:
    target_altitude_m: float = 3.0
    rate_hz: float = 25.0
    duration_s: float = 10.0
    hover_throttle: int = 1660
    kp: float = 20.0
    ki: float = 10.0
    kd: float = 30.0
    velocity_filter_time_constant_s: float = 0.25
    integral_limit_m_s: float = 8.0
    min_throttle: int = 1300
    max_throttle: int = 1850
    throttle_slew_rate_pwm_s: float = 1000.0
    integral_gate_error_m: float = 0.30
    integral_gate_speed_mps: float = 0.50
    prearm_duration_s: float = 3.0
    arming_timeout_s: float = 5.0
    liftoff_height_m: float = 0.05
    liftoff_speed_mps: float = 0.10
    liftoff_timeout_s: float = 3.0
    takeoff_climb_rate_mps: float = 1.0
    takeoff_climb_feedforward_pwm: int = 30
    takeoff_max_lag_m: float = 0.75
    takeoff_ready_dwell_s: float = 0.0
    takeoff_ready_speed_mps: float = 1.0
    takeoff_timeout_s: float = 20.0
    altitude_tolerance_m: float = 0.15
    settle_duration_s: float = 1.0
    settle_timeout_s: float = 10.0
    descent_rate_mps: float = 1.0
    landing_height_m: float = 0.15
    landing_speed_mps: float = 0.15
    landing_settle_s: float = 0.5
    landing_timeout_s: float = 20.0
    max_altitude_error_m: float = 1.0
    telemetry_stale_s: float = 0.30
    status_period_s: float = 0.2
    log_period_s: float = 0.5
    low_throttle: int = 1000
    disarm_burst_s: float = 1.0
    angle_mode: bool = True

    def __post_init__(self) -> None:
        positive = {
            "target_altitude_m": self.target_altitude_m,
            "rate_hz": self.rate_hz,
            "duration_s": self.duration_s,
            "throttle_slew_rate_pwm_s": self.throttle_slew_rate_pwm_s,
            "takeoff_climb_rate_mps": self.takeoff_climb_rate_mps,
            "takeoff_max_lag_m": self.takeoff_max_lag_m,
            "takeoff_ready_speed_mps": self.takeoff_ready_speed_mps,
            "takeoff_timeout_s": self.takeoff_timeout_s,
            "altitude_tolerance_m": self.altitude_tolerance_m,
            "settle_timeout_s": self.settle_timeout_s,
            "descent_rate_mps": self.descent_rate_mps,
            "landing_timeout_s": self.landing_timeout_s,
            "landing_height_m": self.landing_height_m,
            "landing_speed_mps": self.landing_speed_mps,
            "max_altitude_error_m": self.max_altitude_error_m,
            "telemetry_stale_s": self.telemetry_stale_s,
            "status_period_s": self.status_period_s,
            "arming_timeout_s": self.arming_timeout_s,
            "liftoff_height_m": self.liftoff_height_m,
            "liftoff_speed_mps": self.liftoff_speed_mps,
            "liftoff_timeout_s": self.liftoff_timeout_s,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if min(
            self.prearm_duration_s,
            self.takeoff_ready_dwell_s,
            self.settle_duration_s,
            self.landing_settle_s,
        ) < 0.0:
            raise ValueError("phase dwell durations must not be negative")
        if not 800 <= self.low_throttle <= self.min_throttle <= self.max_throttle <= 2200:
            raise ValueError("throttle limits must be ordered within the RC range")
        if self.integral_limit_m_s < 0.0:
            raise ValueError("integral_limit_m_s must not be negative")
        if self.takeoff_climb_feedforward_pwm < 0:
            raise ValueError("takeoff_climb_feedforward_pwm must not be negative")
        if self.velocity_filter_time_constant_s < 0.0:
            raise ValueError("velocity_filter_time_constant_s must not be negative")


@dataclass(frozen=True)
class HoverCycle:
    elapsed_s: float
    phase: str
    target_altitude_m: float
    altitude_m: float
    altitude_error_m: float
    raw_vario_mps: float
    control_velocity_mps: float
    desired_throttle_pwm: int
    sent_throttle_pwm: int
    integral_error_m_s: float
    integral_gate: bool
    armed: bool
    angle_mode: bool


class HoverRecorder(Protocol):
    def record(self, cycle: HoverCycle) -> None: ...

    def mark_landing_confirmed(self) -> None: ...

    def mark_failure(self, reason: str) -> None: ...


class HoverController:
    def __init__(
        self,
        config: HoverConfig,
        telemetry: AltitudeTelemetry,
        status: StatusTelemetry,
        rc_sender: RcSender,
        recorder: HoverRecorder | None = None,
    ) -> None:
        self._config = config
        self._telemetry = telemetry
        self._status = status
        self._rc_sender = rc_sender
        self._recorder = recorder
        self._velocity = VerticalVelocityEstimator(config.velocity_filter_time_constant_s)
        self._pid = AltitudePid(
            config.hover_throttle,
            config.kp,
            config.ki,
            config.kd,
            config.min_throttle,
            config.max_throttle,
            config.integral_limit_m_s,
        )
        self._throttle_limiter = ThrottleSlewLimiter(
            config.throttle_slew_rate_pwm_s,
            float(config.min_throttle),
        )

    def run(self, client: MspClient) -> None:
        loop = RateLoop(self._config.rate_hz)
        mission_started_s = time.monotonic()
        phase = Phase.PREARM
        phase_started_s = mission_started_s
        dwell_started_s: float | None = None
        last_status_s = 0.0
        last_log_s = 0.0
        status: FlightStatus | None = None
        launch_altitude_m: float | None = None
        takeoff_started_s: float | None = None
        descent_start_altitude_m: float | None = None
        abort_reason: str | None = None

        try:
            while True:
                read_started_s = time.monotonic()
                sample = self._telemetry.read(client)
                now_s = time.monotonic()
                vertical_velocity_mps, dt_s = self._velocity.update(sample)
                if status is None or now_s - last_status_s >= self._config.status_period_s:
                    status = self._status.read(client)
                    last_status_s = now_s
                if time.monotonic() - read_started_s > self._config.telemetry_stale_s:
                    raise MissionFailure("MSP telemetry/status response exceeded freshness limit")

                target_m = launch_altitude_m if launch_altitude_m is not None else sample.altitude_m
                desired_throttle = self._config.low_throttle
                sent_throttle = desired_throttle
                integral_gate = False
                armed_command = phase != Phase.PREARM

                if phase == Phase.PREARM:
                    launch_altitude_m = sample.altitude_m
                    if now_s - phase_started_s >= self._config.prearm_duration_s:
                        phase, phase_started_s = self._transition(phase, Phase.ARMING, now_s)

                elif phase == Phase.ARMING:
                    self._timeout(phase, phase_started_s, now_s, self._config.arming_timeout_s)
                    if status.armed and (status.angle_mode or not self._config.angle_mode):
                        self._pid.reset()
                        self._throttle_limiter.reset(self._config.min_throttle)
                        phase, phase_started_s = self._transition(phase, Phase.LIFTOFF, now_s)

                else:
                    if not status.armed:
                        raise MissionFailure("Betaflight unexpectedly disarmed")
                    if self._config.angle_mode and not status.angle_mode:
                        abort_reason = "ANGLE mode became inactive"
                    assert launch_altitude_m is not None
                    final_target_m = launch_altitude_m + self._config.target_altitude_m

                    if abort_reason is not None and phase != Phase.ABORT_DESCEND:
                        descent_start_altitude_m = sample.altitude_m
                        dwell_started_s = None
                        phase, phase_started_s = self._transition(phase, Phase.ABORT_DESCEND, now_s)

                    if phase == Phase.LIFTOFF:
                        target_m = launch_altitude_m
                        desired_throttle, sent_throttle, integral_gate = self._altitude_command(
                            target_m,
                            sample.altitude_m,
                            vertical_velocity_mps,
                            dt_s,
                            feedforward_pwm=self._config.takeoff_climb_feedforward_pwm,
                        )
                        liftoff_confirmed = (
                            sample.altitude_m >= launch_altitude_m + self._config.liftoff_height_m
                            or vertical_velocity_mps >= self._config.liftoff_speed_mps
                        )
                        if liftoff_confirmed:
                            phase, phase_started_s = self._transition(
                                phase, Phase.TAKEOFF, now_s
                            )
                            takeoff_started_s = now_s
                        elif now_s - phase_started_s > self._config.liftoff_timeout_s:
                            abort_reason = "liftoff was not detected before timeout"
                            descent_start_altitude_m = sample.altitude_m
                            phase, phase_started_s = self._transition(
                                phase, Phase.ABORT_DESCEND, now_s
                            )

                    elif phase == Phase.TAKEOFF:
                        assert takeoff_started_s is not None
                        target_m = min(
                            final_target_m,
                            launch_altitude_m
                            + self._config.takeoff_climb_rate_mps * (now_s - takeoff_started_s),
                        )
                        desired_throttle, sent_throttle, integral_gate = self._altitude_command(
                            target_m,
                            sample.altitude_m,
                            vertical_velocity_mps,
                            dt_s,
                            feedforward_pwm=(
                                self._config.takeoff_climb_feedforward_pwm
                                if target_m < final_target_m
                                else 0
                            ),
                        )
                        ready = target_m >= final_target_m and self._altitude_ready(
                            final_target_m, sample.altitude_m, vertical_velocity_mps
                        )
                        dwell_started_s = self._dwell(dwell_started_s, now_s, ready)
                        if target_m - sample.altitude_m > self._config.takeoff_max_lag_m:
                            abort_reason = "vehicle exceeded takeoff ramp lag limit"
                            descent_start_altitude_m = sample.altitude_m
                            dwell_started_s = None
                            phase, phase_started_s = self._transition(
                                phase, Phase.ABORT_DESCEND, now_s
                            )
                        elif dwell_started_s is not None and now_s - dwell_started_s >= self._config.takeoff_ready_dwell_s:
                            dwell_started_s = None
                            phase, phase_started_s = self._transition(phase, Phase.SETTLE, now_s)
                        elif now_s - phase_started_s > self._config.takeoff_timeout_s:
                            abort_reason = "continuous takeoff ramp timed out"
                            descent_start_altitude_m = sample.altitude_m
                            phase, phase_started_s = self._transition(phase, Phase.ABORT_DESCEND, now_s)

                    elif phase == Phase.SETTLE:
                        target_m = final_target_m
                        desired_throttle, sent_throttle, integral_gate = self._altitude_command(
                            target_m, sample.altitude_m, vertical_velocity_mps, dt_s
                        )
                        ready = self._altitude_ready(target_m, sample.altitude_m, vertical_velocity_mps)
                        dwell_started_s = self._dwell(dwell_started_s, now_s, ready)
                        if dwell_started_s is not None and now_s - dwell_started_s >= self._config.settle_duration_s:
                            dwell_started_s = None
                            phase, phase_started_s = self._transition(phase, Phase.SCORED_HOVER, now_s)
                        elif now_s - phase_started_s > self._config.settle_timeout_s:
                            abort_reason = "settle phase timed out"
                            descent_start_altitude_m = sample.altitude_m
                            phase, phase_started_s = self._transition(phase, Phase.ABORT_DESCEND, now_s)

                    elif phase == Phase.SCORED_HOVER:
                        target_m = final_target_m
                        desired_throttle, sent_throttle, integral_gate = self._altitude_command(
                            target_m, sample.altitude_m, vertical_velocity_mps, dt_s
                        )
                        if abs(target_m - sample.altitude_m) > self._config.max_altitude_error_m:
                            abort_reason = "altitude left scored-hover safety envelope"
                            descent_start_altitude_m = sample.altitude_m
                            phase, phase_started_s = self._transition(phase, Phase.ABORT_DESCEND, now_s)
                        elif now_s - phase_started_s >= self._config.duration_s:
                            descent_start_altitude_m = sample.altitude_m
                            dwell_started_s = None
                            phase, phase_started_s = self._transition(phase, Phase.DESCEND, now_s)

                    elif phase in (Phase.DESCEND, Phase.ABORT_DESCEND):
                        assert descent_start_altitude_m is not None
                        if now_s - phase_started_s > self._config.landing_timeout_s:
                            raise MissionFailure(f"{phase.value} landing timed out")
                        target_m = max(
                            launch_altitude_m,
                            descent_start_altitude_m
                            - self._config.descent_rate_mps * (now_s - phase_started_s),
                        )
                        desired_throttle, sent_throttle, integral_gate = self._altitude_command(
                            target_m, sample.altitude_m, vertical_velocity_mps, dt_s
                        )
                        landed = (
                            sample.altitude_m <= launch_altitude_m + self._config.landing_height_m
                            and abs(vertical_velocity_mps) <= self._config.landing_speed_mps
                        )
                        dwell_started_s = self._dwell(dwell_started_s, now_s, landed)
                        if dwell_started_s is not None and now_s - dwell_started_s >= self._config.landing_settle_s:
                            if self._recorder is not None:
                                self._recorder.mark_landing_confirmed()
                            if abort_reason is not None:
                                raise MissionFailure(f"{abort_reason}; controlled landing completed")
                            print("mission: landing confirmed", flush=True)
                            return

                channels = self._channels(sent_throttle, armed_command)
                self._rc_sender.send(client, channels)
                cycle = HoverCycle(
                    elapsed_s=now_s - mission_started_s,
                    phase=phase.value,
                    target_altitude_m=target_m,
                    altitude_m=sample.altitude_m,
                    altitude_error_m=target_m - sample.altitude_m,
                    raw_vario_mps=sample.vertical_velocity_mps,
                    control_velocity_mps=vertical_velocity_mps,
                    desired_throttle_pwm=desired_throttle,
                    sent_throttle_pwm=sent_throttle,
                    integral_error_m_s=self._pid.integral_error_m_s,
                    integral_gate=integral_gate,
                    armed=status.armed,
                    angle_mode=status.angle_mode,
                )
                if self._recorder is not None:
                    self._recorder.record(cycle)
                if now_s - last_log_s >= self._config.log_period_s:
                    self._print_cycle(cycle)
                    last_log_s = now_s
                loop.sleep()
        except Exception as exc:
            if self._recorder is not None:
                self._recorder.mark_failure(str(exc))
            raise
        finally:
            self.disarm(client)

    def disarm(self, client: MspClient) -> None:
        loop = RateLoop(self._config.rate_hz)
        end_s = time.monotonic() + self._config.disarm_burst_s
        channels = self._channels(self._config.low_throttle, False)
        while time.monotonic() < end_s:
            try:
                self._rc_sender.send(client, channels)
            except (ConnectionError, OSError, RuntimeError):
                break
            loop.sleep()
        print("mission: DISARM sent with low throttle", flush=True)

    def _altitude_command(
        self,
        target_m: float,
        altitude_m: float,
        vertical_velocity_mps: float,
        dt_s: float,
        *,
        feedforward_pwm: int = 0,
    ) -> tuple[int, int, bool]:
        integral_gate = (
            abs(target_m - altitude_m) <= self._config.integral_gate_error_m
            and abs(vertical_velocity_mps) <= self._config.integral_gate_speed_mps
        )
        pid_desired = self._pid.command(
            target_m,
            altitude_m,
            vertical_velocity_mps,
            dt_s,
            integrate=integral_gate,
        )
        desired = max(
            self._config.min_throttle,
            min(self._config.max_throttle, pid_desired + feedforward_pwm),
        )
        limiter_dt_s = dt_s if dt_s > 0.0 else 1.0 / self._config.rate_hz
        return desired, self._throttle_limiter.command(desired, limiter_dt_s), integral_gate

    def _altitude_ready(self, target_m: float, altitude_m: float, velocity_mps: float) -> bool:
        return (
            abs(target_m - altitude_m) <= self._config.altitude_tolerance_m
            and abs(velocity_mps) <= self._config.takeoff_ready_speed_mps
        )

    @staticmethod
    def _dwell(started_s: float | None, now_s: float, condition: bool) -> float | None:
        if not condition:
            return None
        return now_s if started_s is None else started_s

    @staticmethod
    def _timeout(phase: Phase, started_s: float, now_s: float, timeout_s: float) -> None:
        if now_s - started_s > timeout_s:
            raise MissionFailure(f"{phase.value} phase timed out")

    @staticmethod
    def _transition(old: Phase, new: Phase, now_s: float) -> tuple[Phase, float]:
        print(f"mission: {old.value} -> {new.value}", flush=True)
        return new, now_s

    def _channels(self, throttle: int, arm: bool) -> RcChannels:
        return RcChannels(
            throttle=throttle,
            aux1=2000 if arm else 1000,
            aux2=2000 if self._config.angle_mode else 1000,
        )

    @staticmethod
    def _print_cycle(cycle: HoverCycle) -> None:
        print(
            f"{cycle.phase}: alt={cycle.altitude_m:.2f}m target={cycle.target_altitude_m:.2f}m "
            f"err={cycle.altitude_error_m:+.2f}m vv={cycle.control_velocity_mps:.2f}m/s "
            f"raw_vv={cycle.raw_vario_mps:.2f}m/s "
            f"pid={cycle.desired_throttle_pwm} throttle={cycle.sent_throttle_pwm} "
            f"i={cycle.integral_error_m_s:.2f} igate={int(cycle.integral_gate)} "
            f"armed={int(cycle.armed)} angle={int(cycle.angle_mode)}",
            flush=True,
        )
