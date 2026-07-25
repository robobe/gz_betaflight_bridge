from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from .geometry import Vec2, yaw_from_quaternion


@dataclass(frozen=True)
class PoseSample:
    xy: Vec2
    z: float
    yaw_rad: float
    timestamp_s: float


class GazeboPoseSource:
    def __init__(self, topic: str, model_name: str = "X3") -> None:
        self._topic = topic
        self._model_name = model_name
        self._node = Node()
        self._lock = threading.Lock()
        self._sample: PoseSample | None = None

    def start(self) -> None:
        if self._topic.endswith("/pose"):
            subscribed = self._node.subscribe(Pose, self._topic, self._on_pose)
        else:
            subscribed = self._node.subscribe(Pose_V, self._topic, self._on_pose_v)

        if not subscribed:
            raise RuntimeError(f"Failed to subscribe to Gazebo pose topic: {self._topic}")

    def latest(self) -> PoseSample | None:
        with self._lock:
            return self._sample

    def wait_for_sample(self, timeout_s: float) -> PoseSample:
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            sample = self.latest()
            if sample is not None:
                return sample
            time.sleep(0.05)
        raise TimeoutError(f"Timed out waiting for Gazebo pose topic: {self._topic}")

    def _on_pose(self, msg: Pose) -> None:
        self._store_pose(msg)

    def _on_pose_v(self, msg: Pose_V) -> None:
        for pose in msg.pose:
            if pose.name == self._model_name:
                self._store_pose(pose)
                return

    def _store_pose(self, msg: Pose) -> None:
        orientation = msg.orientation
        sample = PoseSample(
            xy=Vec2(msg.position.x, msg.position.y),
            z=msg.position.z,
            yaw_rad=yaw_from_quaternion(orientation.w, orientation.x, orientation.y, orientation.z),
            timestamp_s=time.monotonic(),
        )
        with self._lock:
            self._sample = sample
