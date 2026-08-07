from __future__ import annotations

import sys
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from msp_core.telemetry import AltitudeSample, AttitudeSample, FlightStatus  # noqa: E402
from msp_yaw_mission.controller import (  # noqa: E402
    MissionFailure,
    Phase,
    YawCycle,
    YawMissionConfig,
    YawMissionController,
)
from msp_yaw_mission.cli import DEFAULT_CONFIG_PATH, parse_args  # noqa: E402
from msp_yaw_mission.console import BOLD, MAGENTA, RESET, transition_message  # noqa: E402
from msp_yaw_mission.flight_log import YawFlightLog  # noqa: E402
from msp_yaw_mission.yaw_control import YawRateController  # noqa: E402


class YawMissionSequenceTest(unittest.TestCase):
    def test_hover_takeoff_ccw_then_cw_land_and_disarm(self) -> None:
        clock = FakeClock()
        telemetry = SequenceFlightTelemetry(
            clock,
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.1, -90.0),
                (0.1, -180.0),
                (0.1, -180.0),
                (0.1, -90.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.0, 0.0),
            ],
        )
        sender = RecordingSender()
        recorder = RecordingYawLog()
        config = YawMissionConfig(
            target_altitude_m=0.1,
            duration_s=0.04,
            prearm_duration_s=0.0,
            liftoff_height_m=0.05,
            takeoff_climb_rate_mps=10.0,
            takeoff_ready_dwell_s=0.0,
            settle_duration_s=0.0,
            yaw_angle_deg=180.0,
            yaw_rate_dps=15.0,
            yaw_settle_s=0.0,
            yaw_rate_filter_time_constant_s=1000.0,
            velocity_filter_time_constant_s=1000.0,
            descent_rate_mps=10.0,
            landing_settle_s=0.0,
            disarm_burst_s=0.04,
        )
        mission = YawMissionController(
            config,
            telemetry,
            telemetry,
            ArmedAngleStatus(),
            sender,
            recorder,
        )

        with patch("msp_yaw_mission.controller.time.monotonic", clock.monotonic), patch(
            "msp_yaw_mission.controller.RateLoop", lambda rate: FakeRateLoop(clock)
        ):
            mission.run(object())  # type: ignore[arg-type]

        phases = {cycle.phase for cycle in recorder.cycles}
        self.assertIn(Phase.LIFTOFF.value, phases)
        self.assertIn(Phase.YAW_CCW.value, phases)
        self.assertIn(Phase.YAW_CW.value, phases)
        self.assertIn(Phase.DESCEND.value, phases)
        self.assertTrue(any(frame.yaw < 1500 for frame in sender.frames))
        self.assertTrue(any(frame.yaw > 1500 for frame in sender.frames))
        self.assertTrue(recorder.landing_confirmed)
        self.assertEqual(sender.frames[-1].aux1, 1000)

    def test_yaw_timeout_descends_before_reporting_failure_and_disarming(self) -> None:
        clock = FakeClock()
        telemetry = SequenceFlightTelemetry(
            clock,
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.0, 0.0),
            ],
        )
        sender = RecordingSender()
        recorder = RecordingYawLog()
        config = YawMissionConfig(
            target_altitude_m=0.1,
            duration_s=0.04,
            prearm_duration_s=0.0,
            liftoff_height_m=0.05,
            takeoff_climb_rate_mps=10.0,
            takeoff_ready_dwell_s=0.0,
            settle_duration_s=0.0,
            yaw_timeout_s=0.08,
            yaw_rate_filter_time_constant_s=1000.0,
            velocity_filter_time_constant_s=1000.0,
            descent_rate_mps=10.0,
            landing_settle_s=0.0,
            disarm_burst_s=0.04,
        )
        mission = YawMissionController(
            config, telemetry, telemetry, ArmedAngleStatus(), sender, recorder
        )

        with patch("msp_yaw_mission.controller.time.monotonic", clock.monotonic), patch(
            "msp_yaw_mission.controller.RateLoop", lambda rate: FakeRateLoop(clock)
        ):
            with self.assertRaisesRegex(MissionFailure, "controlled landing completed"):
                mission.run(object())  # type: ignore[arg-type]

        self.assertTrue(recorder.landing_confirmed)
        self.assertIn(Phase.ABORT_DESCEND.value, {cycle.phase for cycle in recorder.cycles})
        self.assertEqual(sender.frames[-1].aux1, 1000)


class YawConfigurationTest(unittest.TestCase):
    def test_colocated_defaults_request_five_metres_and_fifteen_degrees_per_second(
        self,
    ) -> None:
        app = parse_args([])

        self.assertEqual(DEFAULT_CONFIG_PATH.parent.name, "msp_yaw_mission")
        self.assertEqual(app.mission.target_altitude_m, 5.0)
        self.assertEqual(app.mission.takeoff_climb_rate_mps, 1.0)
        self.assertEqual(app.mission.yaw_rate_dps, 15.0)
        self.assertEqual(app.mission.yaw_altitude_gate_error_m, 0.30)
        self.assertEqual(app.mission.yaw_altitude_gate_speed_mps, 0.50)


class YawRateControlTest(unittest.TestCase):
    def test_commands_ccw_then_cw_with_rate_feedback(self) -> None:
        controller = YawRateController()

        self.assertLess(controller.command(-180.0, 0.0, -15.0, -1), 1500)
        self.assertGreater(controller.command(0.0, -180.0, 15.0, 1), 1500)
        self.assertEqual(controller.command(-180.0, -178.0, 0.0, -1), 1500)


class PhaseConsoleTest(unittest.TestCase):
    def test_new_phase_is_bold_and_color_coded(self) -> None:
        message = transition_message(Phase.SETTLE, Phase.YAW_CCW, enabled=True)

        self.assertEqual(
            message,
            f"{BOLD}{MAGENTA}mission: settle -> yaw_ccw_180{RESET}",
        )

    def test_color_can_be_disabled_for_redirected_logs(self) -> None:
        message = transition_message(Phase.YAW_CCW, Phase.YAW_CW, enabled=False)

        self.assertEqual(message, "mission: yaw_ccw_180 -> yaw_cw_home")


class YawFlightLogTest(unittest.TestCase):
    def test_external_csv_recorder_captures_yaw_and_landing_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = YawFlightLog(Path(directory), YawMissionConfig())
            log.record(
                YawCycle(
                    elapsed_s=1.0,
                    phase=Phase.YAW_CCW.value,
                    target_altitude_m=5.0,
                    altitude_m=4.9,
                    altitude_error_m=0.1,
                    raw_vario_mps=0.2,
                    control_velocity_mps=0.1,
                    heading_deg=-15.0,
                    target_heading_deg=-180.0,
                    yaw_rate_dps=-14.0,
                    desired_throttle_pwm=1662,
                    sent_throttle_pwm=1661,
                    desired_yaw_pwm=1470,
                    sent_yaw_pwm=1475,
                    integral_error_m_s=0.1,
                    integral_gate=True,
                    armed=True,
                    angle_mode=True,
                )
            )
            log.mark_landing_confirmed()
            log.close()

            summary = json.loads(log.summary_path.read_text(encoding="utf-8"))
            header = log.csv_path.read_text(encoding="utf-8").splitlines()[0]

        self.assertIn("control_velocity_mps", header)
        self.assertIn("yaw_rate_dps", header)
        self.assertEqual(summary["yaw"]["rate_rmse_dps"], 1.0)
        self.assertTrue(summary["safety"]["passed"])


class FakeClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def monotonic(self) -> float:
        return self.now_s


class FakeRateLoop:
    def __init__(self, clock: FakeClock) -> None:
        self._clock = clock

    def sleep(self) -> None:
        self._clock.now_s += 0.04


class SequenceFlightTelemetry:
    def __init__(self, clock: FakeClock, samples: list[tuple[float, float]]) -> None:
        self._clock = clock
        self._samples = iter(samples)
        self._current: tuple[float, float] | None = None

    def read(self, client: object) -> AltitudeSample | AttitudeSample:
        if self._current is None:
            self._current = next(self._samples)
            return AltitudeSample(self._current[0], 0.0, self._clock.now_s)
        current = self._current
        self._current = None
        return AttitudeSample(0.0, 0.0, current[1], self._clock.now_s)


class ArmedAngleStatus:
    def read(self, client: object) -> FlightStatus:
        return FlightStatus(True, True, frozenset({0, 1}), 0.0)


class RecordingSender:
    def __init__(self) -> None:
        self.frames = []

    def send(self, client: object, channels: object) -> None:
        self.frames.append(channels)


class RecordingYawLog:
    def __init__(self) -> None:
        self.cycles: list[YawCycle] = []
        self.landing_confirmed = False
        self.failure: str | None = None

    def record(self, cycle: YawCycle) -> None:
        self.cycles.append(cycle)

    def mark_landing_confirmed(self) -> None:
        self.landing_confirmed = True

    def mark_failure(self, reason: str) -> None:
        self.failure = reason


if __name__ == "__main__":
    unittest.main()
