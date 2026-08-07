from __future__ import annotations

import time
from dataclasses import dataclass

from msp_core.client import MspClient
from msp_core.rc import RcChannels, RcSender
from msp_core.telemetry import AltitudeTelemetry
from msp_core.timing import RateLoop

from .altitude import AltitudeController
from .geometry import Vec2, rotate_body_to_world
from .pose import GazeboPoseSource
from .position import PositionController


@dataclass(frozen=True)
class MissionConfig:
    takeoff_altitude_m: float = 4.0
    start_square_altitude_m: float = 0.0
    square_side_m: float = 6.0
    descent_rate_mps: float = 1.0
    landing_altitude_m: float = 0.20
    position_tolerance_m: float = 0.5
    rate_hz: float = 50.0
    pose_timeout_s: float = 10.0
    max_leg_duration_s: float = 45.0
    max_mission_duration_s: float = 240.0
    prearm_duration_s: float = 3.0
    arm_low_duration_s: float = 1.0
    log_period_s: float = 1.0
    low_throttle: int = 1000
    disarm_burst_s: float = 1.0
    angle_mode: bool = True


class SquareMissionController:
    def __init__(
        self,
        config: MissionConfig,
        pose_source: GazeboPoseSource,
        telemetry: AltitudeTelemetry,
        altitude_controller: AltitudeController,
        position_controller: PositionController,
        rc_sender: RcSender,
    ) -> None:
        self._config = config
        self._pose_source = pose_source
        self._telemetry = telemetry
        self._altitude_controller = altitude_controller
        self._position_controller = position_controller
        self._rc_sender = rc_sender

    def run(self, client: MspClient) -> None:
        self._pose_source.start()
        start_pose = self._pose_source.wait_for_sample(self._config.pose_timeout_s)
        home = start_pose.xy
        mission_yaw = start_pose.yaw_rad
        waypoints = self._square_waypoints(home, mission_yaw)

        loop = RateLoop(self._config.rate_hz)
        start_s = time.monotonic()
        leg_start_s = start_s
        last_log_s = 0.0
        state = "prearm"
        previous_state = ""
        previous_command_key: tuple[str, int, int, int] | None = None
        leg_index = 0
        landing_target_altitude_m = self._config.takeoff_altitude_m

        try:
            while True:
                now_s = time.monotonic()
                elapsed_s = now_s - start_s
                if elapsed_s > self._config.max_mission_duration_s:
                    raise TimeoutError("Mission exceeded max mission duration")

                pose = self._require_recent_pose(now_s)
                altitude = self._telemetry.read(client)
                roll = 1500
                pitch = 1500
                distance_m = 0.0
                speed_command_mps = 0.0
                measured_speed_mps = 0.0
                right_mps = 0.0
                forward_mps = 0.0
                vertical_velocity_mps = altitude.vertical_velocity_mps

                if elapsed_s < self._config.prearm_duration_s:
                    state = "prearm"
                    throttle = self._config.low_throttle
                    arm = False
                elif elapsed_s < self._config.prearm_duration_s + self._config.arm_low_duration_s:
                    state = "arm_low"
                    throttle = self._config.low_throttle
                    arm = True
                elif state in ("prearm", "arm_low", "takeoff"):
                    state = "takeoff"
                    throttle, vertical_velocity_mps = self._altitude_controller.throttle(
                        self._config.takeoff_altitude_m, altitude
                    )
                    arm = True
                    if self._ready_for_square(altitude.altitude_m):
                        state = "leg_forward"
                        leg_start_s = now_s
                        self._position_controller.reset_velocity()
                elif state.startswith("leg_"):
                    if now_s - leg_start_s > self._config.max_leg_duration_s:
                        raise TimeoutError(f"{state} exceeded max leg duration")
                    target = waypoints[leg_index]
                    pos_cmd = self._position_controller.command(target, pose, mission_yaw)
                    roll = pos_cmd.roll
                    pitch = pos_cmd.pitch
                    distance_m = pos_cmd.distance_m
                    speed_command_mps = pos_cmd.speed_command_mps
                    measured_speed_mps = pos_cmd.measured_speed_mps
                    right_mps = pos_cmd.right_mps
                    forward_mps = pos_cmd.forward_mps
                    throttle, vertical_velocity_mps = self._altitude_controller.throttle(
                        self._config.takeoff_altitude_m, altitude
                    )
                    arm = True
                    if distance_m <= self._config.position_tolerance_m:
                        leg_index += 1
                        leg_start_s = now_s
                        self._position_controller.reset_velocity()
                        if leg_index >= len(waypoints):
                            state = "descend"
                        else:
                            state = ("leg_forward", "leg_right", "leg_back", "leg_left")[leg_index]
                elif state == "descend":
                    landing_target_altitude_m = max(
                        0.0,
                        landing_target_altitude_m - self._config.descent_rate_mps / self._config.rate_hz,
                    )
                    throttle, vertical_velocity_mps = self._altitude_controller.throttle(
                        landing_target_altitude_m, altitude
                    )
                    arm = True
                    if altitude.altitude_m <= self._config.landing_altitude_m:
                        break
                else:
                    raise RuntimeError(f"Unexpected mission state: {state}")

                channels = self._channels(roll=roll, pitch=pitch, throttle=throttle, arm=arm)
                self._rc_sender.send(client, channels)

                if state != previous_state:
                    print(
                        f"state: {previous_state or 'start'} -> {state} "
                        f"xy=({pose.xy.x:.2f},{pose.xy.y:.2f}) alt={altitude.altitude_m:.2f}m",
                        flush=True,
                    )
                    previous_state = state
                    previous_command_key = None

                command_name = self._command_name(state)
                command_key = (command_name, round(roll / 25), round(pitch / 25), 0)
                if command_key != previous_command_key:
                    print(
                        f"command: {command_name} "
                        f"target_alt={self._active_target_altitude(state, landing_target_altitude_m):.2f}m "
                        f"target_xy={self._target_xy_text(state, leg_index, waypoints)} "
                        f"roll={roll} pitch={pitch} throttle={throttle} "
                        f"max_xy_speed={self._position_controller.max_horizontal_speed_mps:.2f}m/s",
                        flush=True,
                    )
                    previous_command_key = command_key

                if now_s - last_log_s >= self._config.log_period_s:
                    print(
                        f"{state}: xy=({pose.xy.x:.2f},{pose.xy.y:.2f}) alt={altitude.altitude_m:.2f}m "
                        f"target_alt={self._active_target_altitude(state, landing_target_altitude_m):.2f}m "
                        f"vv={vertical_velocity_mps:.2f}m/s dist={distance_m:.2f}m "
                        f"cmd_speed={speed_command_mps:.2f}m/s measured_speed={measured_speed_mps:.2f}m/s "
                        f"body_cmd=(right={right_mps:.2f},forward={forward_mps:.2f})m/s "
                        f"roll={roll} pitch={pitch} throttle={throttle}",
                        flush=True,
                    )
                    last_log_s = now_s

                loop.sleep()
        finally:
            self.disarm(client)

    def disarm(self, client: MspClient) -> None:
        loop = RateLoop(self._config.rate_hz)
        end_s = time.monotonic() + self._config.disarm_burst_s
        channels = self._channels(roll=1500, pitch=1500, throttle=self._config.low_throttle, arm=False)
        while time.monotonic() < end_s:
            self._rc_sender.send(client, channels)
            loop.sleep()
        angle = 1 if self._config.angle_mode else 0
        print(f"disarmed: throttle=1000 arm=0 angle={angle}", flush=True)

    def _channels(self, roll: int, pitch: int, throttle: int, arm: bool) -> RcChannels:
        return RcChannels(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            aux1=2000 if arm else 1000,
            aux2=2000 if self._config.angle_mode else 1000,
        )

    def _require_recent_pose(self, now_s: float):
        pose = self._pose_source.latest()
        if pose is None:
            raise TimeoutError("No Gazebo pose sample available")
        if now_s - pose.timestamp_s > self._config.pose_timeout_s:
            raise TimeoutError("Gazebo pose sample timed out")
        return pose

    def _square_waypoints(self, home: Vec2, yaw_rad: float) -> list[Vec2]:
        forward = rotate_body_to_world(self._config.square_side_m, 0.0, yaw_rad)
        right = rotate_body_to_world(0.0, -self._config.square_side_m, yaw_rad)
        return [
            home + forward,
            home + forward + right,
            home + right,
            home,
        ]

    def _active_target_altitude(self, state: str, landing_target_altitude_m: float) -> float:
        if state == "descend":
            return landing_target_altitude_m
        return self._config.takeoff_altitude_m

    def _ready_for_square(self, altitude_m: float) -> bool:
        target_reached = abs(self._config.takeoff_altitude_m - altitude_m) <= self._config.position_tolerance_m
        start_altitude_reached = (
            self._config.start_square_altitude_m > 0.0
            and altitude_m >= self._config.start_square_altitude_m
        )
        return target_reached or start_altitude_reached

    @staticmethod
    def _command_name(state: str) -> str:
        names = {
            "prearm": "prearm-disarmed-low-throttle",
            "arm_low": "arm-low-throttle",
            "takeoff": "takeoff-altitude-hold",
            "leg_forward": "forward-position-pid",
            "leg_right": "right-position-pid",
            "leg_back": "back-position-pid",
            "leg_left": "left-position-pid",
            "descend": "descend-and-land",
        }
        return names.get(state, state)

    @staticmethod
    def _target_xy_text(state: str, leg_index: int, waypoints: list[Vec2]) -> str:
        if state.startswith("leg_") and 0 <= leg_index < len(waypoints):
            target = waypoints[leg_index]
            return f"({target.x:.2f},{target.y:.2f})"
        return "none"
