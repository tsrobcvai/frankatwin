"""Minimal quaternion helpers in the API's **wxyz** convention (numpy only).

Everything on the wire -- `RobotState.ee_quat`, `set_ee_target`, `move_to_pose`
-- is wxyz. Sidecars / CSVs / IsaacLab are xyzw; convert at the boundary.
"""

from __future__ import annotations

import numpy as np


def normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    return q / max(float(np.linalg.norm(q)), 1e-12)


def mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a ⊗ b (apply b first, then a)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def conj_wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def from_rotvec_wxyz(rotvec: np.ndarray) -> np.ndarray:
    """Axis-angle vector (rad) -> unit quaternion; exact for small angles."""
    r = np.asarray(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(r))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = r / angle
    return np.concatenate([[np.cos(angle / 2)], axis * np.sin(angle / 2)])


def to_rotvec_wxyz(q: np.ndarray) -> np.ndarray:
    """Unit quaternion -> axis-angle vector (rad), shortest path."""
    q = normalize(q)
    if q[0] < 0.0:
        q = -q
    s = float(np.linalg.norm(q[1:]))
    if s < 1e-12:
        return np.zeros(3)
    return q[1:] / s * (2.0 * np.arctan2(s, q[0]))


def error_rotvec_wxyz(q_des: np.ndarray, q_cur: np.ndarray) -> np.ndarray:
    """Orientation error as osc_shm computes it: 2·vec(q_des ⊗ q_cur⁻¹), shortest path."""
    e = mul_wxyz(normalize(q_des), conj_wxyz(normalize(q_cur)))
    if e[0] < 0.0:
        e = -e
    return 2.0 * e[1:]


def wxyz_to_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([q[1], q[2], q[3], q[0]])


def xyzw_to_wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[3], q[0], q[1], q[2]])
