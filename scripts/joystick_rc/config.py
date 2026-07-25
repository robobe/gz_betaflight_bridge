from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class AxisBinding:
    axis: int
    invert: bool = False
    center: int = 0


@dataclass(frozen=True)
class ButtonBinding:
    button: int
    toggle: bool = False


@dataclass(frozen=True)
class JoystickRcConfig:
    roll: AxisBinding
    pitch: AxisBinding
    throttle: AxisBinding
    yaw: AxisBinding
    arm: ButtonBinding
    angle: ButtonBinding
    deadzone: float = 0.05
    axis_expo: float = 0.35
    throttle_expo: float = 0.35
    min_rc: int = 1000
    mid_rc: int = 1500
    max_rc: int = 2000


def load_config(path: str | Path) -> JoystickRcConfig:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return JoystickRcConfig(
        roll=AxisBinding(**data["roll"]),
        pitch=AxisBinding(**data["pitch"]),
        throttle=AxisBinding(**data["throttle"]),
        yaw=AxisBinding(**data["yaw"]),
        arm=ButtonBinding(**data["arm"]),
        angle=ButtonBinding(**data["angle"]),
        deadzone=data.get("deadzone", 0.05),
        axis_expo=data.get("axis_expo", 0.35),
        throttle_expo=data.get("throttle_expo", 0.35),
        min_rc=data.get("min_rc", 1000),
        mid_rc=data.get("mid_rc", 1500),
        max_rc=data.get("max_rc", 2000),
    )


def save_config(path: str | Path, config: JoystickRcConfig) -> None:
    output = json.dumps(asdict(config), indent=2, sort_keys=True)
    Path(path).write_text(output + "\n", encoding="utf-8")
