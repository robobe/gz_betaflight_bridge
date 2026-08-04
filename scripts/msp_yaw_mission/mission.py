from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from msp_core.client import MspClient
from msp_core.rc import RcChannels, RcSender
from msp_core.telemetry import AltitudeTelemetry, AttitudeTelemetry, FlightStatus, StatusTelemetry
from msp_core.timing import RateLoop

from .console import GREEN, YELLOW, colorize, transition_message
from .control import (
    AltitudePid,
    DirectedYawController,
    ThrottleSlewLimiter,
    VerticalVelocityEstimator,
    YawUnwrapper,
    altitude_steps,
)


class MissionFailure(RuntimeError):
    """Raised when a phase or safety invariant fails."""


class Phase(str, Enum):
    PREARM = "prearm"
    ARMING = "arming"
    TAKEOFF = "takeoff"
    SETTLE = "settle"
    YAW_CW = "yaw_cw_180"
    YAW_CCW = "yaw_ccw_home"
    DESCEND = "descend_1_mps"
    ABORT_DESCEND = "abort_step_descent"


@dataclass(frozen=True)
class MissionConfig:
    rate_hz: float = 25.0
    target_height_m: float = 3.0
    descent_rate_mps: float = 1.0
    hover_throttle: int = 1660
    altitude_kp: float = 20.0
    altitude_ki: float = 10.0
    altitude_kd: float = 30.0
    integral_limit_m_s: float = 8.0
    min_throttle: int = 1300
    max_throttle: int = 1850
    low_throttle: int = 1000
    prearm_duration_s: float = 3.0
    arming_timeout_s: float = 5.0
    takeoff_timeout_s: float = 20.0
    takeoff_step_height_m: float = 0.5
    takeoff_step_dwell_s: float = 0.75
    takeoff_step_speed_mps: float = 0.30
    settle_timeout_s: float = 10.0
    yaw_timeout_s: float = 40.0
    descent_timeout_s: float = 10.0
    altitude_tolerance_m: float = 0.15
    altitude_settle_s: float = 1.0
    yaw_tolerance_deg: float = 5.0
    yaw_settle_s: float = 0.5
    yaw_max_offset_pwm: int = 60
    yaw_min_offset_pwm: int = 20
    yaw_slow_zone_deg: float = 60.0
    yaw_slew_rate_pwm_s: float = 60.0
    yaw_altitude_gate_error_m: float = 0.15
    yaw_altitude_gate_speed_mps: float = 0.30
    yaw_throttle_compensation: float = 0.4
    yaw_clockwise_pwm_sign: int = 1
    direction_check_after_s: float = 1.0
    direction_min_progress_deg: float = 2.0
    landing_height_m: float = 0.15
    landing_speed_mps: float = 0.15
    landing_settle_s: float = 0.5
    max_tilt_deg: float = 30.0
    max_tilt_duration_s: float = 0.2
    max_yaw_altitude_error_m: float = 0.75
    integral_gate_error_m: float = 0.30
    integral_gate_speed_mps: float = 0.50
    throttle_slew_rate_pwm_s: float = 1000.0
    telemetry_stale_s: float = 0.3
    status_period_s: float = 0.2
    log_period_s: float = 0.5
    disarm_burst_s: float = 1.0

    def __post_init__(self) -> None:
        if self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        if self.target_height_m <= 0.0:
            raise ValueError("target_height_m must be positive")
        if self.descent_rate_mps <= 0.0:
            raise ValueError("descent_rate_mps must be positive")
        if self.takeoff_step_height_m <= 0.0:
            raise ValueError("takeoff_step_height_m must be positive")
        if self.takeoff_step_dwell_s < 0.0 or self.takeoff_step_speed_mps <= 0.0:
            raise ValueError("takeoff step dwell and speed gate must be valid")
        if self.throttle_slew_rate_pwm_s <= 0.0:
            raise ValueError("throttle_slew_rate_pwm_s must be positive")
        if not 800 <= self.low_throttle <= self.min_throttle <= self.max_throttle <= 2200:
            raise ValueError("throttle limits must be ordered within the RC range")
        if not 0 < self.yaw_min_offset_pwm <= self.yaw_max_offset_pwm <= 700:
            raise ValueError("yaw PWM offsets must be positive, ordered, and remain within the RC range")
        if self.yaw_slew_rate_pwm_s <= 0.0:
            raise ValueError("yaw_slew_rate_pwm_s must be positive")
        if self.yaw_throttle_compensation < 0.0:
            raise ValueError("yaw_throttle_compensation must not be negative")
        if self.yaw_clockwise_pwm_sign not in (-1, 1):
            raise ValueError("yaw_clockwise_pwm_sign must be -1 or +1")


class YawMission:
    def __init__(
        self,
        config: MissionConfig,
        altitude: AltitudeTelemetry,
        attitude: AttitudeTelemetry,
        status: StatusTelemetry,
        rc_sender: RcSender,
    ) -> None:
        self._config = config
        self._altitude = altitude
        self._attitude = attitude
        self._status = status
        self._rc_sender = rc_sender
        self._pid = AltitudePid(
            config.hover_throttle,
            config.altitude_kp,
            config.altitude_ki,
            config.altitude_kd,
            config.min_throttle,
            config.max_throttle,
            config.integral_limit_m_s,
        )
        self._yaw = DirectedYawController(
            max_offset_pwm=config.yaw_max_offset_pwm,
            min_offset_pwm=config.yaw_min_offset_pwm,
            slow_zone_deg=config.yaw_slow_zone_deg,
            tolerance_deg=config.yaw_tolerance_deg,
        )
        self._throttle_limiter = ThrottleSlewLimiter(
            rate_pwm_s=config.throttle_slew_rate_pwm_s,
            current_pwm=float(config.min_throttle),
        )
        self._yaw_limiter = ThrottleSlewLimiter(
            rate_pwm_s=config.yaw_slew_rate_pwm_s,
            current_pwm=1500.0,
        )

    def run(self, client: MspClient) -> None:
        loop = RateLoop(self._config.rate_hz)
        unwrapper = YawUnwrapper()
        phase = Phase.PREARM
        phase_started_s = time.monotonic()
        dwell_started_s: float | None = None
        tilt_started_s: float | None = None
        last_status_s = 0.0
        last_status: FlightStatus | None = None
        launch_altitude_m: float | None = None
        hover_target_m: float | None = None
        takeoff_targets_m: list[float] = []
        takeoff_step_index = 0
        home_heading_deg: float | None = None
        yaw_target_deg: float | None = None
        yaw_start_heading_deg: float | None = None
        descent_start_altitude_m: float | None = None
        abort_target_m: float | None = None
        abort_reason: str | None = None
        velocity_estimator = VerticalVelocityEstimator()
        last_log_s = 0.0
        armed_confirmed = False

        print("mission: prearm with low throttle and ANGLE enabled", flush=True)
        try:
            while True:
                cycle_started_s = time.monotonic()
                altitude = self._altitude.read(client)
                attitude = self._attitude.read(client)
                now_s = time.monotonic()
                if now_s - cycle_started_s > self._config.telemetry_stale_s:
                    raise MissionFailure("telemetry request cycle exceeded stale-data limit")

                heading_deg = unwrapper.update(attitude.yaw_deg)
                vertical_velocity_mps, altitude_dt_s = velocity_estimator.update(altitude)

                if last_status is None or now_s - last_status_s >= self._config.status_period_s:
                    last_status = self._status.read(client)
                    last_status_s = now_s

                if phase not in (Phase.PREARM, Phase.ARMING):
                    if not last_status.armed:
                        raise MissionFailure("Betaflight unexpectedly disarmed")
                    if not last_status.angle_mode:
                        raise MissionFailure("ANGLE mode is not active")

                    excessive_tilt = max(abs(attitude.roll_deg), abs(attitude.pitch_deg)) > self._config.max_tilt_deg
                    if excessive_tilt:
                        tilt_started_s = tilt_started_s or now_s
                        if now_s - tilt_started_s >= self._config.max_tilt_duration_s:
                            raise MissionFailure("roll/pitch exceeded safety envelope")
                    else:
                        tilt_started_s = None

                throttle = self._config.low_throttle
                desired_throttle = throttle
                yaw_pwm = 1500
                armed = phase != Phase.PREARM
                target_altitude_m = launch_altitude_m or altitude.altitude_m
                integral_gate = False
                altitude_error_m = target_altitude_m - altitude.altitude_m
                step_label = "-"
                dwell_progress_s = 0.0

                if phase == Phase.PREARM:
                    launch_altitude_m = altitude.altitude_m
                    if now_s - phase_started_s >= self._config.prearm_duration_s:
                        hover_target_m = launch_altitude_m + self._config.target_height_m
                        takeoff_targets_m = altitude_steps(
                            launch_altitude_m,
                            self._config.target_height_m,
                            self._config.takeoff_step_height_m,
                        )
                        phase, phase_started_s = self._transition(phase, Phase.ARMING, now_s)

                elif phase == Phase.ARMING:
                    self._check_timeout(phase, phase_started_s, now_s, self._config.arming_timeout_s)
                    if last_status.armed:
                        if not last_status.angle_mode:
                            raise MissionFailure("armed without ANGLE mode active")
                        armed_confirmed = True
                        self._pid.reset()
                        self._throttle_limiter.reset(self._config.min_throttle)
                        phase, phase_started_s = self._transition(phase, Phase.TAKEOFF, now_s)

                elif phase == Phase.TAKEOFF:
                    assert hover_target_m is not None and launch_altitude_m is not None and takeoff_targets_m
                    target_altitude_m = takeoff_targets_m[takeoff_step_index]
                    step_label = f"{takeoff_step_index + 1}/{len(takeoff_targets_m)}"
                    desired_throttle, throttle, integral_gate = self._altitude_command(
                        target_altitude_m,
                        altitude.altitude_m,
                        vertical_velocity_mps,
                        altitude_dt_s,
                    )
                    altitude_error_m = target_altitude_m - altitude.altitude_m
                    step_ready = (
                        abs(altitude_error_m) <= self._config.altitude_tolerance_m
                        and abs(vertical_velocity_mps) <= self._config.takeoff_step_speed_mps
                    )
                    dwell_started_s = self._condition_dwell(dwell_started_s, now_s, step_ready)
                    dwell_progress_s = 0.0 if dwell_started_s is None else now_s - dwell_started_s
                    if dwell_progress_s >= self._config.takeoff_step_dwell_s:
                        completed_step = takeoff_step_index + 1
                        takeoff_step_index += 1
                        dwell_started_s = None
                        if takeoff_step_index >= len(takeoff_targets_m):
                            phase, phase_started_s = self._transition(phase, Phase.SETTLE, now_s)
                        else:
                            phase_started_s = now_s
                            print(
                                colorize(
                                    f"mission: takeoff step {completed_step}/{len(takeoff_targets_m)} complete "
                                    f"-> {takeoff_step_index + 1}/{len(takeoff_targets_m)}",
                                    GREEN,
                                    bold=True,
                                ),
                                flush=True,
                            )
                    elif now_s - phase_started_s > self._config.takeoff_timeout_s:
                        abort_reason = f"takeoff step {step_label} timed out"
                        abort_target_m = max(
                            launch_altitude_m,
                            altitude.altitude_m - self._config.takeoff_step_height_m,
                        )
                        dwell_started_s = None
                        phase, phase_started_s = self._transition(phase, Phase.ABORT_DESCEND, now_s)

                elif phase == Phase.SETTLE:
                    assert hover_target_m is not None and launch_altitude_m is not None
                    if now_s - phase_started_s > self._config.settle_timeout_s:
                        abort_reason = "final hover settle timed out"
                        abort_target_m = max(
                            launch_altitude_m,
                            altitude.altitude_m - self._config.takeoff_step_height_m,
                        )
                        dwell_started_s = None
                        phase, phase_started_s = self._transition(phase, Phase.ABORT_DESCEND, now_s)
                    else:
                        target_altitude_m = hover_target_m
                        desired_throttle, throttle, integral_gate = self._altitude_command(
                            target_altitude_m,
                            altitude.altitude_m,
                            vertical_velocity_mps,
                            altitude_dt_s,
                        )
                        altitude_error_m = target_altitude_m - altitude.altitude_m
                        settled = (
                            abs(altitude_error_m) <= self._config.altitude_tolerance_m
                            and abs(vertical_velocity_mps) <= self._config.takeoff_step_speed_mps
                        )
                        dwell_started_s = self._condition_dwell(dwell_started_s, now_s, settled)
                        dwell_progress_s = 0.0 if dwell_started_s is None else now_s - dwell_started_s
                        if dwell_progress_s >= self._config.altitude_settle_s:
                            home_heading_deg = heading_deg
                            yaw_target_deg = home_heading_deg + 180.0
                            yaw_start_heading_deg = heading_deg
                            dwell_started_s = None
                            phase, phase_started_s = self._transition(phase, Phase.YAW_CW, now_s)

                elif phase == Phase.YAW_CW:
                    assert hover_target_m is not None and yaw_target_deg is not None and yaw_start_heading_deg is not None
                    self._check_timeout(phase, phase_started_s, now_s, self._config.yaw_timeout_s)
                    self._check_yaw_altitude(altitude.altitude_m, hover_target_m)
                    target_altitude_m = hover_target_m
                    desired_throttle, throttle, integral_gate = self._altitude_command(
                        target_altitude_m,
                        altitude.altitude_m,
                        vertical_velocity_mps,
                        altitude_dt_s,
                    )
                    altitude_error_m = target_altitude_m - altitude.altitude_m
                    desired_yaw_pwm = self._map_yaw_pwm(self._yaw.command(yaw_target_deg, heading_deg, 1))
                    yaw_pwm = self._altitude_guarded_yaw(
                        desired_yaw_pwm, altitude_error_m, vertical_velocity_mps, altitude_dt_s
                    )
                    throttle = self._yaw_compensated_throttle(throttle, yaw_pwm)
                    self._check_direction(phase, phase_started_s, now_s, heading_deg - yaw_start_heading_deg)
                    dwell_started_s = self._yaw_dwell(dwell_started_s, now_s, yaw_target_deg, heading_deg)
                    if dwell_started_s is not None and now_s - dwell_started_s >= self._config.yaw_settle_s:
                        assert home_heading_deg is not None
                        yaw_target_deg = home_heading_deg
                        yaw_start_heading_deg = heading_deg
                        dwell_started_s = None
                        phase, phase_started_s = self._transition(phase, Phase.YAW_CCW, now_s)

                elif phase == Phase.YAW_CCW:
                    assert hover_target_m is not None and yaw_target_deg is not None and yaw_start_heading_deg is not None
                    self._check_timeout(phase, phase_started_s, now_s, self._config.yaw_timeout_s)
                    self._check_yaw_altitude(altitude.altitude_m, hover_target_m)
                    target_altitude_m = hover_target_m
                    desired_throttle, throttle, integral_gate = self._altitude_command(
                        target_altitude_m,
                        altitude.altitude_m,
                        vertical_velocity_mps,
                        altitude_dt_s,
                    )
                    altitude_error_m = target_altitude_m - altitude.altitude_m
                    desired_yaw_pwm = self._map_yaw_pwm(self._yaw.command(yaw_target_deg, heading_deg, -1))
                    yaw_pwm = self._altitude_guarded_yaw(
                        desired_yaw_pwm, altitude_error_m, vertical_velocity_mps, altitude_dt_s
                    )
                    throttle = self._yaw_compensated_throttle(throttle, yaw_pwm)
                    self._check_direction(phase, phase_started_s, now_s, yaw_start_heading_deg - heading_deg)
                    dwell_started_s = self._yaw_dwell(dwell_started_s, now_s, yaw_target_deg, heading_deg)
                    if dwell_started_s is not None and now_s - dwell_started_s >= self._config.yaw_settle_s:
                        descent_start_altitude_m = altitude.altitude_m
                        dwell_started_s = None
                        phase, phase_started_s = self._transition(phase, Phase.DESCEND, now_s)

                elif phase == Phase.DESCEND:
                    assert launch_altitude_m is not None and descent_start_altitude_m is not None
                    self._check_timeout(phase, phase_started_s, now_s, self._config.descent_timeout_s)
                    elapsed_s = now_s - phase_started_s
                    target_altitude_m = max(
                        launch_altitude_m,
                        descent_start_altitude_m - self._config.descent_rate_mps * elapsed_s,
                    )
                    desired_throttle, throttle, integral_gate = self._altitude_command(
                        target_altitude_m,
                        altitude.altitude_m,
                        vertical_velocity_mps,
                        altitude_dt_s,
                    )
                    altitude_error_m = target_altitude_m - altitude.altitude_m
                    landed = (
                        altitude.altitude_m <= launch_altitude_m + self._config.landing_height_m
                        and abs(vertical_velocity_mps) <= self._config.landing_speed_mps
                    )
                    dwell_started_s = self._condition_dwell(dwell_started_s, now_s, landed)
                    if dwell_started_s is not None and now_s - dwell_started_s >= self._config.landing_settle_s:
                        print(colorize("mission: landing confirmed", GREEN, bold=True), flush=True)
                        return

                elif phase == Phase.ABORT_DESCEND:
                    assert launch_altitude_m is not None and abort_target_m is not None and abort_reason is not None
                    self._check_timeout(phase, phase_started_s, now_s, self._config.takeoff_timeout_s)
                    target_altitude_m = abort_target_m
                    desired_throttle, throttle, integral_gate = self._altitude_command(
                        target_altitude_m,
                        altitude.altitude_m,
                        vertical_velocity_mps,
                        altitude_dt_s,
                    )
                    altitude_error_m = target_altitude_m - altitude.altitude_m
                    landed = (
                        altitude.altitude_m <= launch_altitude_m + self._config.landing_height_m
                        and abs(vertical_velocity_mps) <= self._config.landing_speed_mps
                    )
                    if abort_target_m <= launch_altitude_m:
                        dwell_started_s = self._condition_dwell(dwell_started_s, now_s, landed)
                        dwell_progress_s = 0.0 if dwell_started_s is None else now_s - dwell_started_s
                        if dwell_progress_s >= self._config.landing_settle_s:
                            raise MissionFailure(f"{abort_reason}; controlled landing completed")
                    else:
                        step_ready = (
                            abs(altitude_error_m) <= self._config.altitude_tolerance_m
                            and abs(vertical_velocity_mps) <= self._config.takeoff_step_speed_mps
                        )
                        dwell_started_s = self._condition_dwell(dwell_started_s, now_s, step_ready)
                        dwell_progress_s = 0.0 if dwell_started_s is None else now_s - dwell_started_s
                        if dwell_progress_s >= self._config.takeoff_step_dwell_s:
                            abort_target_m = max(
                                launch_altitude_m,
                                abort_target_m - self._config.takeoff_step_height_m,
                            )
                            dwell_started_s = None
                            phase_started_s = now_s

                channels = RcChannels(
                    throttle=throttle,
                    yaw=yaw_pwm,
                    aux1=2000 if armed else 1000,
                    aux2=2000,
                    aux3=1000,
                )
                self._rc_sender.send(client, channels)

                if now_s - last_log_s >= self._config.log_period_s:
                    print(
                        f"{phase.value}: alt={altitude.altitude_m:.2f}m target={target_altitude_m:.2f}m "
                        f"err={altitude_error_m:+.2f}m vv={vertical_velocity_mps:.2f}m/s "
                        f"step={step_label} dwell={dwell_progress_s:.2f}s igate={int(integral_gate)} "
                        f"heading={heading_deg:.1f}deg "
                        f"roll={attitude.roll_deg:.1f} pitch={attitude.pitch_deg:.1f} "
                        f"pid={desired_throttle} throttle={throttle} i={self._pid.integral_error_m_s:.2f} "
                        f"yaw={yaw_pwm} armed={int(last_status.armed)}",
                        flush=True,
                    )
                    last_log_s = now_s
                loop.sleep()
        finally:
            self.disarm(client)
            if not armed_confirmed:
                print("mission: arming was never confirmed", flush=True)

    def disarm(self, client: MspClient) -> None:
        loop = RateLoop(self._config.rate_hz)
        end_s = time.monotonic() + self._config.disarm_burst_s
        channels = RcChannels(throttle=self._config.low_throttle, yaw=1500, aux1=1000, aux2=2000, aux3=1000)
        while time.monotonic() < end_s:
            try:
                self._rc_sender.send(client, channels)
            except (ConnectionError, OSError, RuntimeError):
                break
            loop.sleep()
        print(colorize("mission: DISARM sent with low throttle", YELLOW, bold=True), flush=True)

    def _transition(self, old: Phase, new: Phase, now_s: float) -> tuple[Phase, float]:
        print(transition_message(old, new), flush=True)
        return new, now_s

    def _altitude_command(
        self,
        target_altitude_m: float,
        altitude_m: float,
        vertical_velocity_mps: float,
        dt_s: float,
    ) -> tuple[int, int, bool]:
        integral_gate = (
            abs(target_altitude_m - altitude_m) <= self._config.integral_gate_error_m
            and abs(vertical_velocity_mps) <= self._config.integral_gate_speed_mps
        )
        desired_throttle = self._pid.command(
            target_altitude_m,
            altitude_m,
            vertical_velocity_mps,
            dt_s,
            integrate=integral_gate,
        )
        limiter_dt_s = dt_s if dt_s > 0.0 else 1.0 / self._config.rate_hz
        throttle = self._throttle_limiter.command(desired_throttle, limiter_dt_s)
        return desired_throttle, throttle, integral_gate

    @staticmethod
    def _check_timeout(phase: Phase, started_s: float, now_s: float, timeout_s: float) -> None:
        if now_s - started_s > timeout_s:
            raise MissionFailure(f"{phase.value} phase timed out")

    def _check_yaw_altitude(self, altitude_m: float, target_m: float) -> None:
        if abs(altitude_m - target_m) > self._config.max_yaw_altitude_error_m:
            raise MissionFailure("altitude left safety envelope during yaw")

    def _check_direction(self, phase: Phase, started_s: float, now_s: float, progress_deg: float) -> None:
        if now_s - started_s >= self._config.direction_check_after_s and progress_deg < self._config.direction_min_progress_deg:
            raise MissionFailure(f"{phase.value} moved in the wrong direction or made no progress")

    def _map_yaw_pwm(self, logical_pwm: int) -> int:
        return 1500 + self._config.yaw_clockwise_pwm_sign * (logical_pwm - 1500)

    def _altitude_guarded_yaw(
        self,
        desired_yaw_pwm: int,
        altitude_error_m: float,
        vertical_velocity_mps: float,
        dt_s: float,
    ) -> int:
        altitude_stable = (
            abs(altitude_error_m) <= self._config.yaw_altitude_gate_error_m
            and abs(vertical_velocity_mps) <= self._config.yaw_altitude_gate_speed_mps
        )
        if not altitude_stable:
            self._yaw_limiter.reset(1500)
            return 1500
        limiter_dt_s = dt_s if dt_s > 0.0 else 1.0 / self._config.rate_hz
        return self._yaw_limiter.command(desired_yaw_pwm, limiter_dt_s)

    def _yaw_compensated_throttle(self, throttle_pwm: int, yaw_pwm: int) -> int:
        compensation = self._config.yaw_throttle_compensation * abs(yaw_pwm - 1500)
        return int(round(max(self._config.min_throttle, throttle_pwm - compensation)))

    def _yaw_dwell(self, started_s: float | None, now_s: float, target_deg: float, heading_deg: float) -> float | None:
        return self._condition_dwell(
            started_s,
            now_s,
            abs(target_deg - heading_deg) <= self._config.yaw_tolerance_deg,
        )

    @staticmethod
    def _condition_dwell(started_s: float | None, now_s: float, condition: bool) -> float | None:
        if not condition:
            return None
        return now_s if started_s is None else started_s
