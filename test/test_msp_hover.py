from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from msp_hover.cli import DEFAULT_CONFIG_PATH, parse_args  # noqa: E402
from flight_log.csv_recorder import CsvRecorder  # noqa: E402
from msp_core.telemetry import AltitudeSample, FlightStatus  # noqa: E402
from msp_hover.controller import HoverConfig, HoverController, HoverCycle, MissionFailure, Phase  # noqa: E402
from msp_hover.flight_log import HoverFlightLog  # noqa: E402
from msp_hover.tuning import compare_trials  # noqa: E402


class HoverConfigTest(unittest.TestCase):
    def test_colocated_yaml_supplies_pid_and_log_configuration(self) -> None:
        config = parse_args([])

        self.assertEqual(DEFAULT_CONFIG_PATH.name, "msp_hover.yaml")
        self.assertEqual(config.hover.hover_throttle, 1660)
        self.assertEqual(config.hover.kp, 20.0)
        self.assertEqual(config.hover.ki, 10.0)
        self.assertEqual(config.hover.kd, 30.0)
        self.assertEqual(config.hover.velocity_filter_time_constant_s, 0.25)
        self.assertEqual(config.hover.target_altitude_m, 5.0)
        self.assertEqual(config.hover.takeoff_climb_rate_mps, 1.0)
        self.assertEqual(config.hover.takeoff_ready_dwell_s, 0.0)
        self.assertEqual(config.hover.takeoff_ready_speed_mps, 1.0)
        self.assertEqual(config.log_directory, Path("logs/msp-hover"))

    def test_cli_gain_overrides_yaml(self) -> None:
        config = parse_args(["--kp", "25", "--duration", "12"])

        self.assertEqual(config.hover.kp, 25.0)
        self.assertEqual(config.hover.duration_s, 12.0)
        self.assertEqual(config.hover.kd, 30.0)


class HoverMissionSequenceTest(unittest.TestCase):
    def test_liftoff_timeout_aborts_without_starting_altitude_ramp(self) -> None:
        clock = FakeClock()
        recorder = RecordingHoverLog()
        config = HoverConfig(
            prearm_duration_s=0.0,
            liftoff_timeout_s=0.5,
            landing_settle_s=0.0,
            disarm_burst_s=0.0,
        )
        mission = HoverController(
            config,
            TimedAltitude(clock, stop_at_s=2.0),
            ArmedAngleStatus(),
            RecordingSender(),
            recorder,
        )

        with patch("msp_hover.controller.time.monotonic", clock.monotonic), patch(
            "msp_hover.controller.RateLoop", lambda rate: FakeRateLoop(clock)
        ), self.assertRaisesRegex(MissionFailure, "liftoff"):
            mission.run(object())  # type: ignore[arg-type]

        self.assertNotIn(Phase.TAKEOFF.value, {cycle.phase for cycle in recorder.cycles})

    def test_takeoff_target_waits_for_liftoff_evidence(self) -> None:
        clock = FakeClock()
        recorder = RecordingHoverLog()
        config = HoverConfig(
            prearm_duration_s=0.0,
            takeoff_max_lag_m=10.0,
            takeoff_timeout_s=10.0,
            disarm_burst_s=0.0,
        )
        mission = HoverController(
            config,
            TimedAltitude(clock, stop_at_s=1.0),
            ArmedAngleStatus(),
            RecordingSender(),
            recorder,
        )

        with patch("msp_hover.controller.time.monotonic", clock.monotonic), patch(
            "msp_hover.controller.RateLoop", lambda rate: FakeRateLoop(clock)
        ), self.assertRaises(StopReplay):
            mission.run(object())  # type: ignore[arg-type]

        targets = [cycle.target_altitude_m for cycle in recorder.cycles if cycle.armed]
        self.assertEqual(max(targets), 0.0)

    def test_takeoff_aborts_when_vehicle_falls_behind_ramp(self) -> None:
        clock = FakeClock()
        recorder = RecordingHoverLog()
        config = HoverConfig(
            target_altitude_m=5.0,
            prearm_duration_s=0.0,
            takeoff_climb_rate_mps=1.0,
            takeoff_max_lag_m=0.5,
            landing_settle_s=0.0,
            disarm_burst_s=0.0,
        )
        mission = HoverController(
            config,
            DelayedLiftoffAltitude(clock, liftoff_at_s=0.2, stop_at_s=2.0),
            ArmedAngleStatus(),
            RecordingSender(),
            recorder,
        )

        with patch("msp_hover.controller.time.monotonic", clock.monotonic), patch(
            "msp_hover.controller.RateLoop", lambda rate: FakeRateLoop(clock)
        ), self.assertRaisesRegex(MissionFailure, "lag"):
            mission.run(object())  # type: ignore[arg-type]

        self.assertIn(Phase.ABORT_DESCEND.value, {cycle.phase for cycle in recorder.cycles})

    def test_takeoff_target_ramps_continuously_when_vehicle_lags(self) -> None:
        clock = FakeClock()
        telemetry = DelayedLiftoffAltitude(clock, liftoff_at_s=0.2, stop_at_s=1.5)
        recorder = RecordingHoverLog()
        config = HoverConfig(
            target_altitude_m=5.0,
            prearm_duration_s=0.0,
            takeoff_climb_rate_mps=1.0,
            takeoff_climb_feedforward_pwm=30,
            takeoff_max_lag_m=10.0,
            takeoff_ready_dwell_s=0.0,
            takeoff_timeout_s=10.0,
            disarm_burst_s=0.0,
        )
        mission = HoverController(config, telemetry, ArmedAngleStatus(), RecordingSender(), recorder)

        with patch("msp_hover.controller.time.monotonic", clock.monotonic), patch(
            "msp_hover.controller.RateLoop", lambda rate: FakeRateLoop(clock)
        ), self.assertRaises(StopReplay):
            mission.run(object())  # type: ignore[arg-type]

        targets = [cycle.target_altitude_m for cycle in recorder.cycles if cycle.phase == Phase.TAKEOFF]
        desired = [cycle.desired_throttle_pwm for cycle in recorder.cycles if cycle.phase == Phase.TAKEOFF]
        self.assertGreaterEqual(max(targets), 1.0)
        self.assertGreaterEqual(max(desired), config.hover_throttle + config.takeoff_climb_feedforward_pwm)

    def test_confirmed_arm_ramped_takeoff_scored_hover_and_landing(self) -> None:
        clock = FakeClock()
        telemetry = SequenceAltitude(
            clock, [0.0, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
        )
        status = ArmedAngleStatus()
        sender = RecordingSender()
        recorder = RecordingHoverLog()
        config = HoverConfig(
            target_altitude_m=0.1,
            duration_s=0.08,
            prearm_duration_s=0.0,
            takeoff_climb_rate_mps=10.0,
            takeoff_ready_dwell_s=0.0,
            velocity_filter_time_constant_s=1000.0,
            settle_duration_s=0.0,
            descent_rate_mps=10.0,
            landing_settle_s=0.0,
            disarm_burst_s=0.04,
        )
        mission = HoverController(config, telemetry, status, sender, recorder)

        with patch("msp_hover.controller.time.monotonic", clock.monotonic), patch(
            "msp_hover.controller.RateLoop", lambda rate: FakeRateLoop(clock)
        ):
            mission.run(object())  # type: ignore[arg-type]

        phases = {cycle.phase for cycle in recorder.cycles}
        self.assertIn(Phase.TAKEOFF.value, phases)
        self.assertIn(Phase.SCORED_HOVER.value, phases)
        self.assertIn(Phase.DESCEND.value, phases)
        self.assertTrue(recorder.landing_confirmed)
        self.assertEqual(sender.frames[-1].aux1, 1000)


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


class SequenceAltitude:
    def __init__(self, clock: FakeClock, altitudes: list[float]) -> None:
        self._clock = clock
        self._altitudes = iter(altitudes)

    def read(self, client: object) -> AltitudeSample:
        return AltitudeSample(next(self._altitudes), 0.0, self._clock.now_s)


class StopReplay(RuntimeError):
    pass


class TimedAltitude:
    def __init__(self, clock: FakeClock, stop_at_s: float) -> None:
        self._clock = clock
        self._stop_at_s = stop_at_s

    def read(self, client: object) -> AltitudeSample:
        if self._clock.now_s >= self._stop_at_s:
            raise StopReplay
        return AltitudeSample(0.0, 0.0, self._clock.now_s)


class DelayedLiftoffAltitude:
    def __init__(self, clock: FakeClock, liftoff_at_s: float, stop_at_s: float) -> None:
        self._clock = clock
        self._liftoff_at_s = liftoff_at_s
        self._stop_at_s = stop_at_s

    def read(self, client: object) -> AltitudeSample:
        if self._clock.now_s >= self._stop_at_s:
            raise StopReplay
        altitude_m = 0.06 if self._clock.now_s >= self._liftoff_at_s else 0.0
        return AltitudeSample(altitude_m, 0.0, self._clock.now_s)


class ArmedAngleStatus:
    def read(self, client: object) -> FlightStatus:
        return FlightStatus(True, True, frozenset({0, 1}), 0.0)


class RecordingSender:
    def __init__(self) -> None:
        self.frames = []

    def send(self, client: object, channels: object) -> None:
        self.frames.append(channels)


class RecordingHoverLog:
    def __init__(self) -> None:
        self.cycles: list[HoverCycle] = []
        self.landing_confirmed = False
        self.failure: str | None = None

    def record(self, cycle: HoverCycle) -> None:
        self.cycles.append(cycle)

    def mark_landing_confirmed(self) -> None:
        self.landing_confirmed = True

    def mark_failure(self, reason: str) -> None:
        self.failure = reason


class HoverFlightLogTest(unittest.TestCase):
    def test_writes_full_rate_csv_and_summary_metrics(self) -> None:
        config = HoverConfig(target_altitude_m=3.0, min_throttle=1300, max_throttle=1850)
        with tempfile.TemporaryDirectory() as directory:
            recorder = HoverFlightLog(Path(directory), config)
            recorder.record(self._cycle(4.0, Phase.SCORED_HOVER, 0.5, 0.5, 1700))
            recorder.record(self._cycle(4.1, Phase.SCORED_HOVER, -0.2, -0.1, 1600))
            recorder.record(self._cycle(4.2, Phase.DESCEND, -0.1, -0.2, 1580))
            recorder.mark_landing_confirmed()
            recorder.close()

            rows = recorder.csv_path.read_text(encoding="utf-8").splitlines()
            summary = json.loads(recorder.summary_path.read_text(encoding="utf-8"))

        self.assertEqual(len(rows), 4)
        self.assertIn("raw_vario_mps", rows[0])
        self.assertEqual(summary["steady_hover"]["samples"], 2)
        self.assertAlmostEqual(summary["steady_hover"]["mae_m"], 0.35)
        self.assertGreater(summary["steady_hover"]["oscillations_per_minute"], 0.0)
        self.assertTrue(summary["safety"]["passed"])

    def test_rejects_unsafe_throttle_order(self) -> None:
        with self.assertRaisesRegex(ValueError, "throttle limits"):
            HoverConfig(low_throttle=1400, min_throttle=1300)

    @staticmethod
    def _cycle(
        elapsed_s: float,
        phase: Phase,
        error_m: float,
        velocity_mps: float,
        throttle_pwm: int,
    ) -> HoverCycle:
        target_m = 3.0
        return HoverCycle(
            elapsed_s=elapsed_s,
            phase=phase.value,
            target_altitude_m=target_m,
            altitude_m=target_m - error_m,
            altitude_error_m=error_m,
            raw_vario_mps=velocity_mps * 2.0,
            control_velocity_mps=velocity_mps,
            desired_throttle_pwm=throttle_pwm,
            sent_throttle_pwm=throttle_pwm,
            integral_error_m_s=0.1,
            integral_gate=True,
            armed=True,
            angle_mode=True,
        )


class ReusableCsvRecorderTest(unittest.TestCase):
    def test_records_arbitrary_mapping_schema(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.csv"
            with CsvRecorder(path, ["state", "value"]) as recorder:
                recorder.write({"state": "ready", "value": 42})

            rows = path.read_text(encoding="utf-8").splitlines()

        self.assertEqual(rows, ["state,value", "ready,42"])


class HoverTuningComparisonTest(unittest.TestCase):
    def test_accepts_repeated_candidate_without_guarded_regression(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = [self._summary(root / f"baseline-{index}.json", 0.20, 0.30) for index in range(3)]
            candidate = [self._summary(root / f"candidate-{index}.json", 0.15, 0.31) for index in range(3)]

            comparison = compare_trials(baseline, candidate)

        self.assertTrue(comparison.accepted)

    def test_rejects_candidate_with_more_oscillation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = [self._summary(root / "baseline.json", 0.20, 0.30, oscillation=3.0)]
            candidate = [self._summary(root / "candidate.json", 0.15, 0.30, oscillation=8.0)]

            comparison = compare_trials(baseline, candidate)

        self.assertFalse(comparison.accepted)
        self.assertIn("oscillation", comparison.reason)

    @staticmethod
    def _summary(
        path: Path,
        rmse: float,
        max_error: float,
        *,
        oscillation: float = 3.0,
    ) -> Path:
        document = {
            "safety": {"passed": True},
            "steady_hover": {
                "rmse_m": rmse,
                "max_abs_error_m": max_error,
                "vertical_speed_rms_mps": 0.2,
                "throttle_saturation_percent": 0.0,
                "oscillations_per_minute": oscillation,
            },
        }
        path.write_text(json.dumps(document), encoding="utf-8")
        return path


if __name__ == "__main__":
    unittest.main()
