"""Shared, deterministic pose sequence for the real arm and Isaac Lab."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import json
from pathlib import Path

import numpy as np

from .math import multiply, rotvec_quat, slerp


@dataclass
class Protocol:
    mass_kg: float = 0.310
    dimensions_m: list = field(default_factory=lambda: [0.05, 0.05, 0.10])
    ee_com_m: list = field(default_factory=lambda: [0.0, 0.0, 0.025])
    q_init: list = field(
        default_factory=lambda: [
            0.0,
            -np.pi / 4,
            0.0,
            -3 * np.pi / 4,
            0.0,
            np.pi / 2,
            np.pi / 4,
        ]
    )
    rotation_waypoints_deg: list = field(
        default_factory=lambda: [
            [0, 0, 0],
            [30, 0, 0],
            [-30, 0, 0],
            [0, 30, 0],
            [0, -30, 0],
            [20, 20, 0],
            [-20, -20, 0],
            [0, 0, 30],
            [0, 0, 0],
        ]
    )
    move_seconds: float = 4.0
    hold_seconds: float = 2.0
    settle_seconds: float = 0.7
    control_hz: float = 50.0
    kp_pos: float = 500.0
    kp_ori: float = 30.0
    joint_kp: list = field(default_factory=lambda: [400., 400., 400., 400., 120., 100., 60.])
    joint_kd: list = field(default_factory=lambda: [40., 40., 40., 40., 14., 12., 8.])
    max_joint_speed: float = 0.6
    max_joint_acceleration: float = 1.5
    grasp_force_n: float = 35.0
    gripper_width_tolerance_m: float = 0.004

    def validate(self):
        for name in (
            "mass_kg",
            "move_seconds",
            "hold_seconds",
            "control_hz",
            "kp_pos",
            "kp_ori",
            "max_joint_speed",
            "max_joint_acceleration",
        ):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.control_hz < 10 or self.max_joint_speed > 2:
            raise ValueError("joint mode requires control_hz >= 10 and max_joint_speed <= 2 rad/s")
        for name, size in (("dimensions_m", 3), ("ee_com_m", 3), ("q_init", 7)):
            a = np.asarray(getattr(self, name), float)
            if a.shape != (size,) or not np.all(np.isfinite(a)):
                raise ValueError(f"invalid {name}")
        for name, limit in (("joint_kp", 1000), ("joint_kd", 100)):
            value = np.asarray(getattr(self, name), float)
            if value.shape != (7,) or not np.all(np.isfinite(value)) or np.any(value <= 0) or np.any(value > limit):
                raise ValueError(f"invalid {name}")
        if np.any(np.asarray(self.dimensions_m) <= 0) or self.dimensions_m[1] >= 0.08:
            raise ValueError(
                "positive dimensions required; grasp width must be below 80 mm"
            )
        if not 0 <= self.settle_seconds < self.hold_seconds:
            raise ValueError("settle_seconds must be in [0, hold_seconds)")
        if (
            not 30 <= self.grasp_force_n <= 70
            or not 0 < self.gripper_width_tolerance_m < 0.01
        ):
            raise ValueError("invalid gripper force or width tolerance")
        wp = np.asarray(self.rotation_waypoints_deg, float)
        if (
            wp.ndim != 2
            or wp.shape[1] != 3
            or len(wp) < 2
            or not np.all(np.isfinite(wp))
        ):
            raise ValueError("at least two rotation waypoints are required")
        if not np.allclose(wp[0], 0) or not np.allclose(wp[-1], 0):
            raise ValueError(
                "the sequence must start and end at its anchor orientation"
            )
        return self

    @property
    def duration(self):
        return (
            len(self.rotation_waypoints_deg) * self.hold_seconds
            + (len(self.rotation_waypoints_deg) - 1) * self.move_seconds
        )

    def sample(self, t, anchor_pos, anchor_quat):
        """Return position, wxyz, waypoint index, and settled-hold eligibility."""
        t = float(np.clip(t, 0, self.duration))
        q0 = np.asarray(anchor_quat, float)
        quats = [
            multiply(q0, rotvec_quat(np.deg2rad(v)))
            for v in self.rotation_waypoints_deg
        ]
        if t <= self.hold_seconds:
            return np.array(anchor_pos), quats[0], 0, t >= self.settle_seconds
        t -= self.hold_seconds
        for i in range(1, len(quats)):
            if t < self.move_seconds:
                u = t / self.move_seconds
                s = 10 * u**3 - 15 * u**4 + 6 * u**5
                return np.array(anchor_pos), slerp(quats[i - 1], quats[i], s), i, False
            t -= self.move_seconds
            if t <= self.hold_seconds or i == len(quats) - 1:
                return np.array(anchor_pos), quats[i], i, t >= self.settle_seconds
            t -= self.hold_seconds
        raise RuntimeError("unreachable trajectory segment")

    def to_dict(self):
        return asdict(self)

    @property
    def digest(self):
        return hashlib.sha256(
            json.dumps(self.to_dict(), sort_keys=True).encode()
        ).hexdigest()

    @classmethod
    def load(cls, path=None):
        return (
            cls(**json.loads(Path(path).read_text())).validate()
            if path
            else cls().validate()
        )
