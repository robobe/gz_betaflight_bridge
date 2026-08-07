from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from flight_control.altitude import AltitudePid, ThrottleSlewLimiter, VerticalVelocityEstimator
from msp_core.client import MspClient
from msp_core.rc import RcChannels, RcSender
from msp_core.telemetry import AltitudeTelemetry, AttitudeTelemetry, FlightStatus, StatusTelemetry
from msp_core.timing import RateLoop
from msp_hover.controller import HoverConfig

from .console import transition_message
from .yaw_control import YawRateController, YawRateEstimator, YawUnwrapper


class MissionFailure(RuntimeError):
    """Raised after a safety or mission failure."""


class Phase(str, Enum):
    PREARM = "prearm"
    ARMING = "arming"
    LIFTOFF = "liftoff"
    TAKEOFF = "takeoff"
    SETTLE = "settle"
    YAW_CCW = "yaw_ccw_180"
    YAW_CW = "yaw_cw_home"
    DESCEND = "descend"
    ABORT_DESCEND = "abort_descend"


@dataclass(frozen=True)
class YawMissionConfig(HoverConfig):
    yaw_angle_deg: float = 180.0
    yaw_rate_dps: float = 15.0
    yaw_rate_feedforward_pwm: int = 20
    yaw_rate_kp_pwm_per_dps: float = 2.0
    yaw_rate_filter_time_constant_s: float = 0.15
    yaw_max_offset_pwm: int = 60
    yaw_slow_zone_deg: float = 60.0
    yaw_tolerance_deg: float = 5.0
    yaw_settle_s: float = 0.5
    yaw_timeout_s: float = 40.0
    yaw_slew_rate_pwm_s: float = 60.0
    yaw_altitude_gate_error_m: float = 0.30
    yaw_altitude_gate_speed_mps: float = 0.50
    max_yaw_altitude_error_m: float = 0.75
    yaw_clockwise_pwm_sign: int = 1

    def __post_init__(self) -> None:
        super().__post_init__()
        positive = {
            "yaw_angle_deg": self.yaw_angle_deg,
            "yaw_rate_dps": self.yaw_rate_dps,
            "yaw_timeout_s": self.yaw_timeout_s,
            "yaw_slew_rate_pwm_s": self.yaw_slew_rate_pwm_s,
            "yaw_altitude_gate_error_m": self.yaw_altitude_gate_error_m,
            "yaw_altitude_gate_speed_mps": self.yaw_altitude_gate_speed_mps,
            "max_yaw_altitude_error_m": self.max_yaw_altitude_error_m,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.yaw_rate_filter_time_constant_s < 0.0 or self.yaw_rate_kp_pwm_per_dps < 0.0:
            raise ValueError("yaw filter and rate gain must not be negative")
        if min(self.yaw_settle_s, self.yaw_slow_zone_deg, self.yaw_tolerance_deg) < 0.0:
            raise ValueError("yaw settle, slow zone, and tolerance must not be negative")
        if not 0 <= self.yaw_rate_feedforward_pwm <= self.yaw_max_offset_pwm <= 700:
            raise ValueError("yaw PWM feed-forward and maximum must be ordered")
        if self.yaw_clockwise_pwm_sign not in (-1, 1):
            raise ValueError("yaw_clockwise_pwm_sign must be -1 or +1")


@dataclass(frozen=True)
class YawCycle:
    elapsed_s: float
    phase: str
    target_altitude_m: float
    altitude_m: float
    altitude_error_m: float
    raw_vario_mps: float
    control_velocity_mps: float
    heading_deg: float
    target_heading_deg: float
    yaw_rate_dps: float
    desired_throttle_pwm: int
    sent_throttle_pwm: int
    desired_yaw_pwm: int
    sent_yaw_pwm: int
    integral_error_m_s: float
    integral_gate: bool
    armed: bool
    angle_mode: bool


class YawRecorder(Protocol):
    def record(self, cycle: YawCycle) -> None: ...

    def mark_landing_confirmed(self) -> None: ...

    def mark_failure(self, reason: str) -> None: ...


class YawMissionController:
    def __init__(
        self,
        config: YawMissionConfig,
        altitude: AltitudeTelemetry,
        attitude: AttitudeTelemetry,
        status: StatusTelemetry,
        rc_sender: RcSender,
        recorder: YawRecorder | None = None,
    ) -> None:
        self._config = config
        self._altitude = altitude
        self._attitude = attitude
        self._status = status
        self._rc_sender = rc_sender
        self._recorder = recorder
        self._altitude_velocity = VerticalVelocityEstimator(
            config.velocity_filter_time_constant_s
        )
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
            config.throttle_slew_rate_pwm_s, float(config.min_throttle)
        )
        self._yaw = YawRateController(
            config.yaw_rate_dps,
            config.yaw_rate_feedforward_pwm,
            config.yaw_rate_kp_pwm_per_dps,
            config.yaw_max_offset_pwm,
            config.yaw_slow_zone_deg,
            config.yaw_tolerance_deg,
        )
        self._yaw_limiter = ThrottleSlewLimiter(config.yaw_slew_rate_pwm_s, 1500.0)

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
        home_heading_deg: float | None = None
        yaw_target_deg: float | None = None
        abort_reason: str | None = None
        unwrapper = YawUnwrapper()
        yaw_velocity = YawRateEstimator(self._config.yaw_rate_filter_time_constant_s)

        try:
            while True:
                read_started_s = time.monotonic()
                altitude = self._altitude.read(client)
                attitude = self._attitude.read(client)
                now_s = time.monotonic()
                vertical_velocity_mps, dt_s = self._altitude_velocity.update(altitude)
                heading_deg = unwrapper.update(attitude.yaw_deg)
                yaw_rate_dps, _ = yaw_velocity.update(heading_deg, attitude.timestamp_s)
                if status is None or now_s - last_status_s >= self._config.status_period_s:
                    status = self._status.read(client)
                    last_status_s = now_s
                if time.monotonic() - read_started_s > self._config.telemetry_stale_s:
                    raise MissionFailure("MSP telemetry/status response exceeded freshness limit")

                target_m = launch_altitude_m if launch_altitude_m is not None else altitude.altitude_m
                target_heading_deg = heading_deg
                desired_throttle = self._config.low_throttle
                sent_throttle = desired_throttle
                desired_yaw = 1500
                sent_yaw = 1500
                integral_gate = False
                armed_command = phase != Phase.PREARM

                if phase == Phase.PREARM:
                    launch_altitude_m = altitude.altitude_m
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
                        descent_start_altitude_m = altitude.altitude_m
                        dwell_started_s = None
                        phase, phase_started_s = self._transition(
                            phase, Phase.ABORT_DESCEND, now_s
                        )

                    if phase == Phase.LIFTOFF:
                        target_m = launch_altitude_m
                        desired_throttle, sent_throttle, integral_gate = self._altitude_command(
                            target_m,
                            altitude.altitude_m,
                            vertical_velocity_mps,
                            dt_s,
                            feedforward_pwm=self._config.takeoff_climb_feedforward_pwm,
                        )
                        liftoff = (
                            altitude.altitude_m
                            >= launch_altitude_m + self._config.liftoff_height_m
                            or vertical_velocity_mps >= self._config.liftoff_speed_mps
                        )
                        if liftoff:
                            phase, phase_started_s = self._transition(
                                phase, Phase.TAKEOFF, now_s
                            )
                            takeoff_started_s = now_s
                        elif now_s - phase_started_s > self._config.liftoff_timeout_s:
                            abort_reason = "liftoff was not detected before timeout"
                            descent_start_altitude_m = altitude.altitude_m
                            phase, phase_started_s = self._transition(
                                phase, Phase.ABORT_DESCEND, now_s
                            )

                    elif phase == Phase.TAKEOFF:
                        assert takeoff_started_s is not None
                        target_m = min(
                            final_target_m,
                            launch_altitude_m
                            + self._config.takeoff_climb_rate_mps
                            * (now_s - takeoff_started_s),
                        )
                        desired_throttle, sent_throttle, integral_gate = self._altitude_command(
                            target_m,
                            altitude.altitude_m,
                            vertical_velocity_mps,
                            dt_s,
                            feedforward_pwm=(
                                self._config.takeoff_climb_feedforward_pwm
                                if target_m < final_target_m
                                else 0
                            ),
                        )
                        ready = target_m >= final_target_m and self._altitude_ready(
                            final_target_m, altitude.altitude_m, vertical_velocity_mps
                        )
                        dwell_started_s = self._dwell(dwell_started_s, now_s, ready)
                        if target_m - altitude.altitude_m > self._config.takeoff_max_lag_m:
                            abort_reason = "vehicle exceeded takeoff ramp lag limit"
                            descent_start_altitude_m = altitude.altitude_m
                            dwell_started_s = None
                            phase, phase_started_s = self._transition(
                                phase, Phase.ABORT_DESCEND, now_s
                            )
                        elif (
                            dwell_started_s is not None
                            and now_s - dwell_started_s
                            >= self._config.takeoff_ready_dwell_s
                        ):
                            dwell_started_s = None
                            phase, phase_started_s = self._transition(
                                phase, Phase.SETTLE, now_s
                            )
                        elif now_s - phase_started_s > self._config.takeoff_timeout_s:
                            abort_reason = "continuous takeoff ramp timed out"
                            descent_start_altitude_m = altitude.altitude_m
                            phase, phase_started_s = self._transition(
                                phase, Phase.ABORT_DESCEND, now_s
                            )

                    elif phase == Phase.SETTLE:
                        target_m = final_target_m
                        desired_throttle, sent_throttle, integral_gate = self._altitude_command(
                            target_m, altitude.altitude_m, vertical_velocity_mps, dt_s
                        )
                        ready = self._altitude_ready(
                            target_m, altitude.altitude_m, vertical_velocity_mps
                        )
                        dwell_started_s = self._dwell(dwell_started_s, now_s, ready)
                        if (
                            dwell_started_s is not None
                            and now_s - dwell_started_s >= self._config.settle_duration_s
                        ):
                            home_heading_deg = heading_deg
                            yaw_target_deg = home_heading_deg - self._config.yaw_angle_deg
                            dwell_started_s = None
                            phase, phase_started_s = self._transition(
                                phase, Phase.YAW_CCW, now_s
                            )
                        elif now_s - phase_started_s > self._config.settle_timeout_s:
                            abort_reason = "settle phase timed out"
                            descent_start_altitude_m = altitude.altitude_m
                            phase, phase_started_s = self._transition(
                                phase, Phase.ABORT_DESCEND, now_s
                            )

                    elif phase in (Phase.YAW_CCW, Phase.YAW_CW):
                        assert home_heading_deg is not None and yaw_target_deg is not None
                        if now_s - phase_started_s > self._config.yaw_timeout_s:
                            abort_reason = f"{phase.value} phase timed out"
                            descent_start_altitude_m = altitude.altitude_m
                            phase, phase_started_s = self._transition(
                                phase, Phase.ABORT_DESCEND, now_s
                            )
                        else:
                            target_m = final_target_m
                            target_heading_deg = yaw_target_deg
                            desired_throttle, sent_throttle, integral_gate = (
                                self._altitude_command(
                                    target_m,
                                    altitude.altitude_m,
                                    vertical_velocity_mps,
                                    dt_s,
                                )
                            )
                            altitude_error_m = target_m - altitude.altitude_m
                            if (
                                abs(altitude_error_m)
                                > self._config.max_yaw_altitude_error_m
                            ):
                                abort_reason = "altitude left safety envelope during yaw"
                                descent_start_altitude_m = altitude.altitude_m
                                phase, phase_started_s = self._transition(
                                    phase, Phase.ABORT_DESCEND, now_s
                                )
                            else:
                                direction = -1 if phase == Phase.YAW_CCW else 1
                                logical_yaw = self._yaw.command(
                                    yaw_target_deg, heading_deg, yaw_rate_dps, direction
                                )
                                desired_yaw = self._map_yaw_pwm(logical_yaw)
                                sent_yaw = self._altitude_guarded_yaw(
                                    desired_yaw,
                                    altitude_error_m,
                                    vertical_velocity_mps,
                                    dt_s,
                                )
                                ready = (
                                    abs(yaw_target_deg - heading_deg)
                                    <= self._config.yaw_tolerance_deg
                                )
                                dwell_started_s = self._dwell(
                                    dwell_started_s, now_s, ready
                                )
                                if (
                                    dwell_started_s is not None
                                    and now_s - dwell_started_s
                                    >= self._config.yaw_settle_s
                                ):
                                    dwell_started_s = None
                                    if phase == Phase.YAW_CCW:
                                        yaw_target_deg = home_heading_deg
                                        phase, phase_started_s = self._transition(
                                            phase, Phase.YAW_CW, now_s
                                        )
                                    else:
                                        descent_start_altitude_m = altitude.altitude_m
                                        phase, phase_started_s = self._transition(
                                            phase, Phase.DESCEND, now_s
                                        )

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
                            target_m, altitude.altitude_m, vertical_velocity_mps, dt_s
                        )
                        landed = (
                            altitude.altitude_m
                            <= launch_altitude_m + self._config.landing_height_m
                            and abs(vertical_velocity_mps) <= self._config.landing_speed_mps
                        )
                        dwell_started_s = self._dwell(dwell_started_s, now_s, landed)
                        if (
                            dwell_started_s is not None
                            and now_s - dwell_started_s >= self._config.landing_settle_s
                        ):
                            if self._recorder is not None:
                                self._recorder.mark_landing_confirmed()
                            if abort_reason is not None:
                                raise MissionFailure(
                                    f"{abort_reason}; controlled landing completed"
                                )
                            print("mission: landing confirmed", flush=True)
                            return

                channels = RcChannels(
                    throttle=sent_throttle,
                    yaw=sent_yaw,
                    aux1=2000 if armed_command else 1000,
                    aux2=2000 if self._config.angle_mode else 1000,
                )
                self._rc_sender.send(client, channels)
                cycle = YawCycle(
                    elapsed_s=now_s - mission_started_s,
                    phase=phase.value,
                    target_altitude_m=target_m,
                    altitude_m=altitude.altitude_m,
                    altitude_error_m=target_m - altitude.altitude_m,
                    raw_vario_mps=altitude.vertical_velocity_mps,
                    control_velocity_mps=vertical_velocity_mps,
                    heading_deg=heading_deg,
                    target_heading_deg=target_heading_deg,
                    yaw_rate_dps=yaw_rate_dps,
                    desired_throttle_pwm=desired_throttle,
                    sent_throttle_pwm=sent_throttle,
                    desired_yaw_pwm=desired_yaw,
                    sent_yaw_pwm=sent_yaw,
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
        channels = self._channels(self._config.low_throttle, 1500, False)
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

    def _altitude_guarded_yaw(
        self,
        desired_yaw_pwm: int,
        altitude_error_m: float,
        vertical_velocity_mps: float,
        dt_s: float,
    ) -> int:
        stable = (
            abs(altitude_error_m) <= self._config.yaw_altitude_gate_error_m
            and abs(vertical_velocity_mps) <= self._config.yaw_altitude_gate_speed_mps
        )
        if not stable:
            self._yaw_limiter.reset(1500)
            return 1500
        limiter_dt_s = dt_s if dt_s > 0.0 else 1.0 / self._config.rate_hz
        return self._yaw_limiter.command(desired_yaw_pwm, limiter_dt_s)

    def _map_yaw_pwm(self, logical_pwm: int) -> int:
        return 1500 + self._config.yaw_clockwise_pwm_sign * (logical_pwm - 1500)

    def _channels(self, throttle: int, yaw: int, arm: bool) -> RcChannels:
        return RcChannels(
            throttle=throttle,
            yaw=yaw,
            aux1=2000 if arm else 1000,
            aux2=2000 if self._config.angle_mode else 1000,
        )

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
        print(transition_message(old, new), flush=True)
        return new, now_s

    @staticmethod
    def _print_cycle(cycle: YawCycle) -> None:
        print(
            f"{cycle.phase}: alt={cycle.altitude_m:.2f}m target={cycle.target_altitude_m:.2f}m "
            f"err={cycle.altitude_error_m:+.2f}m vv={cycle.control_velocity_mps:.2f}m/s "
            f"raw_vv={cycle.raw_vario_mps:.2f}m/s heading={cycle.heading_deg:.1f}deg "
            f"target_heading={cycle.target_heading_deg:.1f}deg "
            f"yaw_rate={cycle.yaw_rate_dps:.1f}deg/s throttle={cycle.sent_throttle_pwm} "
            f"yaw={cycle.sent_yaw_pwm} armed={int(cycle.armed)} angle={int(cycle.angle_mode)}",
            flush=True,
        )
