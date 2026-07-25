from __future__ import annotations

from dataclasses import dataclass

from .geometry import Vec2, world_to_body_right_forward
from .pose import PoseSample


@dataclass(frozen=True)
class PositionCommand:
    roll: int
    pitch: int
    distance_m: float
    speed_command_mps: float
    measured_speed_mps: float
    right_mps: float
    forward_mps: float
    desired_velocity_world: Vec2


@dataclass
class PositionController:
    kp_position: float
    ki_position: float
    kd_velocity: float
    integral_limit: float
    max_horizontal_speed_mps: float
    rc_us_per_mps: float
    roll_min: int
    roll_max: int
    pitch_min: int
    pitch_max: int
    pitch_forward_sign: int
    roll_right_sign: int
    _previous_pose: PoseSample | None = None
    _integral_error: Vec2 = Vec2(0.0, 0.0)

    def command(self, target_xy: Vec2, pose: PoseSample, mission_yaw_rad: float) -> PositionCommand:
        velocity_world, dt_s = self._derive_velocity(pose)
        error = target_xy - pose.xy
        self._integral_error = (self._integral_error + error * dt_s).clamp_norm(self.integral_limit)
        desired_velocity = (
            error * self.kp_position
            + self._integral_error * self.ki_position
            - velocity_world * self.kd_velocity
        ).clamp_norm(
            self.max_horizontal_speed_mps
        )
        right_mps, forward_mps = world_to_body_right_forward(desired_velocity, mission_yaw_rad)
        roll = self._clamp_rc(1500 + self.roll_right_sign * right_mps * self.rc_us_per_mps, self.roll_min, self.roll_max)
        pitch = self._clamp_rc(
            1500 + self.pitch_forward_sign * forward_mps * self.rc_us_per_mps,
            self.pitch_min,
            self.pitch_max,
        )
        return PositionCommand(
            roll=roll,
            pitch=pitch,
            distance_m=error.norm(),
            speed_command_mps=desired_velocity.norm(),
            measured_speed_mps=velocity_world.norm(),
            right_mps=right_mps,
            forward_mps=forward_mps,
            desired_velocity_world=desired_velocity,
        )

    def reset_velocity(self) -> None:
        self._previous_pose = None
        self._integral_error = Vec2(0.0, 0.0)

    def _derive_velocity(self, pose: PoseSample) -> tuple[Vec2, float]:
        previous = self._previous_pose
        self._previous_pose = pose
        if previous is None:
            return Vec2(0.0, 0.0), 0.0

        dt_s = pose.timestamp_s - previous.timestamp_s
        if dt_s <= 0.0:
            return Vec2(0.0, 0.0), 0.0

        delta = pose.xy - previous.xy
        return delta * (1.0 / dt_s), dt_s

    @staticmethod
    def _clamp_rc(value: float, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, int(round(value))))
