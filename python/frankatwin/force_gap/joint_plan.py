"""Nominal Panda kinematics and a shared, bounded joint reference for force-gap runs."""
from __future__ import annotations

import hashlib
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .math import pose_matrix, matrix_quaternion, quaternion_angle, validate_transform

Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
TAU_LIMIT = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])


def flange_fk(q):
    """Panda modified-DH chain, ending at the flange (107 mm after joint 7)."""
    transform = np.eye(4)
    for theta, a, d, alpha in zip(
        q,
        [0, 0, 0, 0.0825, -0.0825, 0, 0.088],
        [0.333, 0, 0.316, 0, 0.384, 0, 0],
        [0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2],
    ):
        c, s, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
        transform = transform @ np.array(
            [
                [c, -s, 0, a],
                [s * ca, c * ca, -sa, -d * sa],
                [s * sa, c * sa, ca, d * ca],
                [0, 0, 0, 1],
            ]
        )
    flange = np.eye(4)
    flange[2, 3] = 0.107
    return transform @ flange


def default_ee_transform():
    """Bundled Panda hand orientation and fingertip-center flange offset."""
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("z", -np.pi / 4).as_matrix()
    result[2, 3] = 0.112071
    return result


def joint_torque(q, dq, target_q, target_dq, kp, kd, coriolis, previous, dt):
    """Joint PD + Coriolis, then magnitude and 1000 Nm/s rate limits; excludes gravity."""
    tau = np.asarray(kp) * (target_q - q) + np.asarray(kd) * (target_dq - dq) + coriolis
    return np.clip(
        np.clip(tau, -TAU_LIMIT, TAU_LIMIT), previous - 1000 * dt, previous + 1000 * dt
    )


class JointPlan:
    def __init__(self, t, q, f_t_ee, anchor_pos, anchor_quat, metrics=None):
        self.t, self.q = np.asarray(t), np.asarray(q)
        self.f_t_ee = validate_transform(f_t_ee)
        self.anchor_pos, self.anchor_quat = np.asarray(anchor_pos), np.asarray(
            anchor_quat
        )
        self.spline = CubicSpline(
            self.t, self.q, bc_type=((1, np.zeros(7)), (1, np.zeros(7)))
        )
        self.metrics = metrics or {}

    def sample(self, t):
        time = np.clip(t, self.t[0], self.t[-1])
        return self.spline(time), self.spline(time, 1)

    @property
    def digest(self):
        return hashlib.sha256(
            b"".join(
                np.asarray(v, dtype="<f8").tobytes()
                for v in (
                    self.t,
                    self.q,
                    self.f_t_ee,
                    self.anchor_pos,
                    self.anchor_quat,
                )
            )
        ).hexdigest()

    def save(self, path):
        np.savez_compressed(
            path,
            t_s=self.t,
            q=self.q,
            dq=self.spline(self.t, 1),
            F_T_EE=self.f_t_ee,
            anchor_pos=self.anchor_pos,
            anchor_quat=self.anchor_quat,
        )

    @classmethod
    def build(cls, protocol, f_t_ee, anchor_pos=None, anchor_quat=None):
        q_init = np.asarray(protocol.q_init, float)
        f_t_ee = validate_transform(f_t_ee)
        initial = flange_fk(q_init) @ f_t_ee
        anchor_pos = initial[:3, 3] if anchor_pos is None else np.asarray(anchor_pos)
        anchor_quat = (
            matrix_quaternion(initial[:3, :3])
            if anchor_quat is None
            else np.asarray(anchor_quat)
        )
        if (
            np.linalg.norm(initial[:3, 3] - anchor_pos) > 0.003
            or quaternion_angle(matrix_quaternion(initial[:3, :3]), anchor_quat) > 0.015
        ):
            raise ValueError(
                "nominal Panda FK disagrees with measured anchor; check robot model and F_T_EE"
            )
        # Hold the exact reset configuration. The relative desired EE path is
        # anchored to its nominal FK; measured-vs-nominal discrepancy is reported.
        anchor_error = float(np.linalg.norm(initial[:3, 3] - anchor_pos))
        anchor_pos = initial[:3, 3]
        anchor_quat = matrix_quaternion(initial[:3, :3])
        t = np.linspace(
            0, protocol.duration, round(protocol.duration * protocol.control_hz) + 1
        )
        qs, previous = [], q_init.copy()
        max_pos_error, max_rot_error = 0.0, 0.0
        for time in t:
            pos, quat, _, _ = protocol.sample(time, anchor_pos, anchor_quat)
            goal = pose_matrix(pos, quat)

            def residual(q):
                current = flange_fk(q) @ f_t_ee
                return np.r_[
                    current[:3, 3] - pos,
                    0.3
                    * Rotation.from_matrix(
                        goal[:3, :3] @ current[:3, :3].T
                    ).as_rotvec(),
                    0.0003 * (q - q_init),
                ]

            solution = least_squares(
                residual,
                previous,
                bounds=(Q_MIN + 0.08, Q_MAX - 0.08),
                ftol=1e-10,
                xtol=1e-10,
                gtol=1e-10,
                max_nfev=80,
            )
            if not solution.success:
                raise ValueError(
                    f"IK did not converge at {time:.3f}s: {solution.message}"
                )
            previous = solution.x
            qs.append(previous.copy())
            current = flange_fk(previous) @ f_t_ee
            max_pos_error = max(
                max_pos_error, float(np.linalg.norm(current[:3, 3] - pos))
            )
            max_rot_error = max(
                max_rot_error,
                float(quaternion_angle(matrix_quaternion(current[:3, :3]), quat)),
            )
        plan = cls(t, qs, f_t_ee, anchor_pos, anchor_quat)
        dense_t = np.linspace(0, protocol.duration, round(protocol.duration * 1000) + 1)
        dense_q, dense_dq = plan.sample(dense_t)
        max_speed = float(np.abs(dense_dq).max())
        max_acc = float(np.abs(plan.spline(dense_t, 2)).max())
        if max_pos_error > 0.0002 or max_rot_error > 0.001:
            raise ValueError(
                f"IK EE residual too large: {max_pos_error} m, {max_rot_error} rad"
            )
        if np.any(dense_q < Q_MIN + 0.05) or np.any(dense_q > Q_MAX - 0.05):
            raise ValueError("interpolated joint plan violates joint-limit margin")
        if (
            max_speed > protocol.max_joint_speed
            or max_acc > protocol.max_joint_acceleration
        ):
            raise ValueError(
                f"joint plan exceeds speed/acceleration limits: {max_speed:.4f} rad/s, {max_acc:.4f} rad/s^2; increase move_seconds"
            )
        if np.max(np.abs(plan.q[-1] - q_init)) > 0.002:
            raise ValueError("joint plan does not return to the reset posture")
        plan.metrics = dict(
            max_ik_position_error_m=max_pos_error,
            max_ik_orientation_error_rad=max_rot_error,
            max_speed_rad_s=max_speed,
            max_acceleration_rad_s2=max_acc,
            measured_anchor_position_error_m=anchor_error,
            max_joint_excursion_deg=np.rad2deg(
                np.max(np.abs(dense_q - q_init), axis=0)
            ).tolist(),
        )
        return plan
