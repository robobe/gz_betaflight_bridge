from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scale: float) -> Vec2:
        return Vec2(self.x * scale, self.y * scale)

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def clamp_norm(self, limit: float) -> Vec2:
        length = self.norm()
        if length <= limit or length <= 1e-9:
            return self
        scale = limit / length
        return self * scale


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_body_to_world(forward_m: float, left_m: float, yaw_rad: float) -> Vec2:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return Vec2(
        forward_m * cos_yaw - left_m * sin_yaw,
        forward_m * sin_yaw + left_m * cos_yaw,
    )


def world_to_body_right_forward(world: Vec2, yaw_rad: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    forward = world.x * cos_yaw + world.y * sin_yaw
    left = -world.x * sin_yaw + world.y * cos_yaw
    right = -left
    return right, forward
