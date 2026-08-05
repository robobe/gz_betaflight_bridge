from __future__ import annotations

import sys
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from msp_yaw_mission.control import (  # noqa: E402
    AltitudePid,
    DirectedYawController,
    ThrottleSlewLimiter,
    VerticalVelocityEstimator,
    YawUnwrapper,
    altitude_steps,
)
from msp_core.telemetry import AltitudeSample, AttitudeSample, FlightStatus  # noqa: E402
from msp_yaw_mission.console import RESET, transition_message  # noqa: E402
from msp_yaw_mission.mission import MissionConfig, MissionFailure, Phase, YawMission  # noqa: E402


class YawControlTest(unittest.TestCase):
    def test_transition_message_uses_destination_color(self) -> None:
        message = transition_message(Phase.ARMING, Phase.TAKEOFF, enabled=True)

        self.assertIn("\033[", message)
        self.assertIn("mission: arming -> takeoff", message)
        self.assertTrue(message.endswith(RESET))

    def test_transition_message_is_plain_when_color_is_disabled(self) -> None:
        self.assertEqual(
            transition_message(Phase.PREARM, Phase.ARMING, enabled=False),
            "mission: prearm -> arming",
        )

    def test_unwraps_heading_across_360_degrees(self) -> None:
        unwrapper = YawUnwrapper()

        self.assertEqual(unwrapper.update(350.0), 350.0)
        self.assertEqual(unwrapper.update(359.0), 359.0)
        self.assertEqual(unwrapper.update(2.0), 362.0)
        self.assertEqual(unwrapper.update(355.0), 355.0)

    def test_direction_constrained_yaw_commands(self) -> None:
        controller = DirectedYawController()

        self.assertEqual(controller.command(180.0, 0.0, 1), 1700)
        self.assertEqual(controller.command(180.0, 178.0, 1), 1500)
        self.assertEqual(controller.command(0.0, 180.0, -1), 1300)
        self.assertEqual(controller.command(0.0, 2.0, -1), 1500)

    def test_yaw_command_reduces_authority_near_target(self) -> None:
        controller = DirectedYawController()

        self.assertEqual(controller.command(180.0, 150.0, 1), 1633)
        self.assertEqual(controller.command(180.0, 175.0, 1), 1500)

    def test_altitude_pid_clamps_throttle(self) -> None:
        pid = AltitudePid(1600, 100.0, 0.0, 90.0, 1100, 2000, 5.0)

        self.assertEqual(pid.command(10.0, 0.0, 0.0, 0.04), 2000)
        self.assertEqual(pid.command(0.0, 10.0, 0.0, 0.04), 1100)

    def test_altitude_pid_does_not_wind_up_while_saturated(self) -> None:
        pid = AltitudePid(1600, 100.0, 10.0, 90.0, 1100, 2000, 8.0)

        for _ in range(100):
            self.assertEqual(pid.command(20.0, 0.0, 0.0, 0.04), 2000)

        self.assertEqual(pid.integral_error_m_s, 0.0)

    def test_altitude_pid_freezes_integral_outside_gate(self) -> None:
        pid = AltitudePid(1600, 50.0, 10.0, 150.0, 1300, 1850, 8.0)

        pid.command(3.0, 0.0, 2.0, 0.04, integrate=False)

        self.assertEqual(pid.integral_error_m_s, 0.0)

    def test_throttle_slew_limiter_bounds_each_change(self) -> None:
        limiter = ThrottleSlewLimiter(rate_pwm_s=150.0, current_pwm=1300.0)

        self.assertEqual(limiter.command(1850, 0.04), 1306)
        self.assertEqual(limiter.command(1850, 0.04), 1312)
        self.assertEqual(limiter.command(1300, 0.04), 1306)

    def test_builds_launch_relative_altitude_steps(self) -> None:
        self.assertEqual(altitude_steps(0.2, 3.0, 0.5), [0.7, 1.2, 1.7, 2.2, 2.7, 3.2])
        self.assertEqual(altitude_steps(0.0, 1.2, 0.5), [0.5, 1.0, 1.2])

    def test_first_step_demand_exceeds_observed_lift_threshold(self) -> None:
        config = MissionConfig()
        pid = AltitudePid(
            config.hover_throttle,
            config.altitude_kp,
            config.altitude_ki,
            config.altitude_kd,
            config.min_throttle,
            config.max_throttle,
            config.integral_limit_m_s,
        )

        demand = pid.command(0.5, 0.0, 0.0, 0.04, integrate=False)

        self.assertGreater(demand, 1660)

    def test_velocity_estimator_uses_reported_vario_for_held_altitude_samples(self) -> None:
        estimator = VerticalVelocityEstimator()

        estimator.update(AltitudeSample(1.0, 1.2, 1.0))
        velocity_mps, dt_s = estimator.update(AltitudeSample(1.5, 1.1, 1.04))

        self.assertAlmostEqual(velocity_mps, 1.1)
        self.assertAlmostEqual(dt_s, 0.04)

    def test_default_altitude_control_reaches_and_holds_target(self) -> None:
        config = MissionConfig()
        pid = AltitudePid(
            config.hover_throttle,
            config.altitude_kp,
            config.altitude_ki,
            config.altitude_kd,
            config.min_throttle,
            config.max_throttle,
            config.integral_limit_m_s,
        )
        altitude_m = 0.0
        velocity_mps = 0.0
        dt_s = 1.0 / config.rate_hz
        settled_samples = 0
        max_settled_samples = 0

        control_window_s = config.takeoff_timeout_s + config.settle_timeout_s
        for _ in range(round(control_window_s * config.rate_hz)):
            throttle = pid.command(3.0, altitude_m, velocity_mps, dt_s)
            acceleration_mps2 = 0.015 * (throttle - 1660) - 1.5 * velocity_mps
            velocity_mps += acceleration_mps2 * dt_s
            altitude_m = max(0.0, altitude_m + velocity_mps * dt_s)
            if abs(altitude_m - 3.0) <= config.altitude_tolerance_m:
                settled_samples += 1
            else:
                settled_samples = 0
            max_settled_samples = max(max_settled_samples, settled_samples)

        self.assertGreaterEqual(max_settled_samples, round(config.altitude_settle_s * config.rate_hz))

    def test_altitude_control_handles_stepwise_msp_altitude_updates(self) -> None:
        config = MissionConfig()
        pid = AltitudePid(
            config.hover_throttle,
            config.altitude_kp,
            config.altitude_ki,
            config.altitude_kd,
            config.min_throttle,
            config.max_throttle,
            config.integral_limit_m_s,
        )
        estimator = VerticalVelocityEstimator()
        altitude_m = 0.0
        velocity_mps = 0.0
        measured_altitude_m = 0.0
        dt_s = 1.0 / config.rate_hz
        settled_samples = 0

        for index in range(round(config.takeoff_timeout_s * config.rate_hz)):
            if index % 5 == 0:
                measured_altitude_m = round(altitude_m, 2)
            reported_vario_mps = velocity_mps
            sample = AltitudeSample(measured_altitude_m, reported_vario_mps, index * dt_s)
            estimated_velocity_mps, controller_dt_s = estimator.update(sample)
            throttle = pid.command(3.0, measured_altitude_m, estimated_velocity_mps, controller_dt_s)
            acceleration_mps2 = 0.15 * (throttle - 1660) - 0.5 * velocity_mps
            velocity_mps += acceleration_mps2 * dt_s
            altitude_m = max(0.0, altitude_m + velocity_mps * dt_s)
            if abs(measured_altitude_m - 3.0) <= config.altitude_tolerance_m:
                settled_samples += 1
            else:
                settled_samples = 0

        self.assertGreaterEqual(settled_samples, round(config.altitude_settle_s * config.rate_hz))

    def test_default_control_handles_delayed_thrust_response(self) -> None:
        config = MissionConfig()
        pid = AltitudePid(
            config.hover_throttle,
            config.altitude_kp,
            config.altitude_ki,
            config.altitude_kd,
            config.min_throttle,
            config.max_throttle,
            config.integral_limit_m_s,
        )
        altitude_m = 0.0
        velocity_mps = 0.0
        limiter = ThrottleSlewLimiter(
            rate_pwm_s=config.throttle_slew_rate_pwm_s,
            current_pwm=float(config.min_throttle),
        )
        effective_throttle = float(config.min_throttle)
        dt_s = 1.0 / config.rate_hz
        first_in_band_s = None
        settled_at_s = None
        dwell_s = 0.0
        max_altitude_m = 0.0

        target_m = config.takeoff_step_height_m
        for index in range(round(config.takeoff_timeout_s * config.rate_hz)):
            error_m = target_m - altitude_m
            integrate = (
                abs(error_m) <= config.integral_gate_error_m
                and abs(velocity_mps) <= config.integral_gate_speed_mps
            )
            desired_throttle = pid.command(target_m, altitude_m, velocity_mps, dt_s, integrate=integrate)
            throttle = limiter.command(desired_throttle, dt_s)
            effective_throttle += (throttle - effective_throttle) * dt_s / 0.2
            acceleration_mps2 = 0.15 * (effective_throttle - 1660) - 0.5 * velocity_mps
            velocity_mps += acceleration_mps2 * dt_s
            altitude_m = max(0.0, altitude_m + velocity_mps * dt_s)
            max_altitude_m = max(max_altitude_m, altitude_m)

            inside = (
                abs(altitude_m - target_m) <= config.altitude_tolerance_m
                and abs(velocity_mps) <= config.takeoff_step_speed_mps
            )
            if inside and first_in_band_s is None:
                first_in_band_s = index * dt_s
            dwell_s = dwell_s + dt_s if inside else 0.0
            if dwell_s >= config.altitude_settle_s:
                settled_at_s = index * dt_s
                break

        self.assertLessEqual(max_altitude_m, 0.8)
        self.assertIsNotNone(first_in_band_s)
        self.assertIsNotNone(settled_at_s)
        assert first_in_band_s is not None and settled_at_s is not None
        self.assertLessEqual(settled_at_s - first_in_band_s, config.settle_timeout_s)


class MissionSafetyTest(unittest.TestCase):
    def setUp(self) -> None:
        config = MissionConfig()
        self.mission = YawMission(config, object(), object(), object(), object())  # type: ignore[arg-type]

    def test_condition_dwell_resets_when_condition_is_false(self) -> None:
        self.assertEqual(self.mission._condition_dwell(None, 10.0, True), 10.0)
        self.assertEqual(self.mission._condition_dwell(10.0, 10.2, True), 10.0)
        self.assertIsNone(self.mission._condition_dwell(10.0, 10.2, False))

    def test_rejects_unsafe_configuration(self) -> None:
        with self.assertRaisesRegex(ValueError, "descent_rate_mps"):
            MissionConfig(descent_rate_mps=0.0)

    def test_phase_timeout_is_strictly_enforced(self) -> None:
        with self.assertRaisesRegex(MissionFailure, "yaw_cw_180 phase timed out"):
            self.mission._check_timeout(Phase.YAW_CW, 0.0, 10.1, 10.0)

    def test_yaw_altitude_escape_is_rejected(self) -> None:
        with self.assertRaisesRegex(MissionFailure, "altitude left safety envelope"):
            self.mission._check_yaw_altitude(3.76, 3.0)

    def test_yaw_is_slew_limited_while_altitude_is_stable(self) -> None:
        self.assertEqual(self.mission._altitude_guarded_yaw(1560, 0.05, 0.10, 0.04), 1502)

    def test_yaw_returns_to_center_when_altitude_is_not_stable(self) -> None:
        self.mission._altitude_guarded_yaw(1560, 0.05, 0.10, 0.04)

        self.assertEqual(self.mission._altitude_guarded_yaw(1560, -0.16, 0.10, 0.04), 1500)

    def test_wrong_direction_is_rejected(self) -> None:
        with self.assertRaisesRegex(MissionFailure, "wrong direction"):
            self.mission._check_direction(Phase.YAW_CW, 0.0, 1.0, -3.0)

    def test_reversed_yaw_polarity_maps_clockwise_below_center(self) -> None:
        reversed_mission = YawMission(
            MissionConfig(yaw_clockwise_pwm_sign=-1),
            object(),
            object(),
            object(),
            object(),
        )  # type: ignore[arg-type]

        self.assertEqual(reversed_mission._map_yaw_pwm(1700), 1300)
        self.assertEqual(reversed_mission._map_yaw_pwm(1300), 1700)


class SequenceTelemetry:
    def __init__(self, samples: list[object]) -> None:
        self._samples = iter(samples)

    def read(self, client: object) -> object:
        return next(self._samples)


class ArmedAngleStatus:
    def read(self, client: object) -> FlightStatus:
        return FlightStatus(True, True, frozenset({0, 1}), 0.0)


class RecordingRcSender:
    def __init__(self) -> None:
        self.frames = []

    def send(self, client: object, channels: object) -> None:
        self.frames.append(channels)


class MissionSequenceTest(unittest.TestCase):
    def test_runs_complete_mission_sequence(self) -> None:
        altitudes = [0.0, 0.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 0.0, 0.0]
        headings = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 179.0, 179.0, 90.0, 0.0, 0.0, 0.0]
        altitude_samples = [
            AltitudeSample(value, 0.0, index * 0.04) for index, value in enumerate(altitudes)
        ]
        attitude_samples = [
            AttitudeSample(0.0, 0.0, value, index * 0.04) for index, value in enumerate(headings)
        ]
        rc_sender = RecordingRcSender()
        mission = YawMission(
            MissionConfig(
                rate_hz=1_000_000.0,
                prearm_duration_s=0.0,
                takeoff_step_height_m=3.0,
                takeoff_step_dwell_s=0.0,
                altitude_settle_s=0.0,
                yaw_settle_s=0.0,
                landing_settle_s=0.0,
                disarm_burst_s=0.0,
            ),
            SequenceTelemetry(altitude_samples),  # type: ignore[arg-type]
            SequenceTelemetry(attitude_samples),  # type: ignore[arg-type]
            ArmedAngleStatus(),  # type: ignore[arg-type]
            rc_sender,  # type: ignore[arg-type]
        )

        with redirect_stdout(StringIO()):
            mission.run(object())  # type: ignore[arg-type]

        self.assertTrue(any(frame.aux1 == 1000 for frame in rc_sender.frames))
        self.assertTrue(any(frame.aux1 == 2000 for frame in rc_sender.frames))
        self.assertTrue(any(frame.yaw > 1500 for frame in rc_sender.frames))
        self.assertTrue(any(frame.yaw < 1500 for frame in rc_sender.frames))


if __name__ == "__main__":
    unittest.main()
