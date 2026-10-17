"""Wrench transforms and rigid-body support loads, in SI units.

Wrenches use [Fx,Fy,Fz,Tx,Ty,Tz]. The canonical sign is robot-on-payload,
matching libfranka's documented O_F_ext_hat_K sign. Quaternions are wxyz.
"""
from __future__ import annotations

import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    if not np.all(np.isfinite(q)) or np.any(norm < 1e-10):
        raise ValueError("invalid quaternion")
    w, x, y, z = np.moveaxis(q / norm, -1, 0)
    return np.stack(
        [
            1 - 2 * (y * y + z * z),
            2 * (x * y - z * w),
            2 * (x * z + y * w),
            2 * (x * y + z * w),
            1 - 2 * (x * x + z * z),
            2 * (y * z - x * w),
            2 * (x * z - y * w),
            2 * (y * z + x * w),
            1 - 2 * (x * x + y * y),
        ],
        axis=-1,
    ).reshape(q.shape[:-1] + (3, 3))


def multiply(a, b):
    a, b = np.broadcast_arrays(np.asarray(a, float), np.asarray(b, float))
    aw, av, bw, bv = a[..., :1], a[..., 1:], b[..., :1], b[..., 1:]
    return np.concatenate(
        [
            aw * bw - np.sum(av * bv, axis=-1, keepdims=True),
            aw * bv + bw * av + np.cross(av, bv),
        ],
        axis=-1,
    )


def rotvec_quat(v):
    v = np.asarray(v, float)
    angle = np.linalg.norm(v, axis=-1, keepdims=True)
    return np.concatenate(
        [np.cos(angle / 2), 0.5 * np.sinc(angle / (2 * np.pi)) * v], axis=-1
    )


def slerp(q0, q1, u):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
    dot = float(q0 @ q1)
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = (1 - u) * q0 + u * q1
        return q / np.linalg.norm(q)
    angle = np.arccos(np.clip(dot, -1, 1))
    return (np.sin((1 - u) * angle) * q0 + np.sin(u * angle) * q1) / np.sin(angle)


def pose_matrix(pos, quat):
    r = rotation(quat)
    out = np.broadcast_to(np.eye(4), r.shape[:-2] + (4, 4)).copy()
    out[..., :3, :3], out[..., :3, 3] = r, pos
    return out


def matrix_quaternion(r):
    """Convert a single proper 3x3 rotation to a wxyz quaternion."""
    r = np.asarray(r, float)
    trace = np.trace(r)
    if trace > 0:
        s = 2 * np.sqrt(trace + 1)
        q = np.array(
            [
                s / 4,
                (r[2, 1] - r[1, 2]) / s,
                (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s,
            ]
        )
    else:
        i = int(np.argmax(np.diag(r)))
        j = (i + 1) % 3
        k = (i + 2) % 3
        s = 2 * np.sqrt(max(0.0, 1 + r[i, i] - r[j, j] - r[k, k]))
        q = np.zeros(4)
        q[0] = (r[k, j] - r[j, k]) / s
        q[i + 1] = s / 4
        q[j + 1] = (r[j, i] + r[i, j]) / s
        q[k + 1] = (r[k, i] + r[i, k]) / s
    return q / np.linalg.norm(q)


def validate_transform(t):
    t = np.asarray(t, float)
    if t.shape != (4, 4) or not np.all(np.isfinite(t)):
        raise ValueError("transform must be a finite 4x4 matrix")
    if (
        not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6)
        or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
        or np.linalg.det(t[:3, :3]) < 0
    ):
        raise ValueError(
            "transform must contain a proper rotation and homogeneous last row"
        )
    return t


def transform_wrench(wrench, rotation_target_source, source_minus_target):
    """Change axes AND moment origin.

    source_minus_target is p_source - p_target expressed in target axes.
    This is not a pose transform of a six-element position vector.
    """
    w = np.asarray(wrench, float)
    r = np.asarray(rotation_target_source, float)
    f = np.einsum("...ij,...j->...i", r, w[..., :3])
    tau = np.einsum("...ij,...j->...i", r, w[..., 3:]) + np.cross(
        source_minus_target, f
    )
    return np.concatenate([f, tau], axis=-1)


def box_inertia(mass, dimensions):
    x, y, z = np.asarray(dimensions, float)
    return np.diag(np.array([y * y + z * z, x * x + z * z, x * x + y * y]) * mass / 12)


def support_wrench(
    mass,
    com_pos,
    reference_pos,
    com_acc=None,
    rotation_world_body=None,
    inertia_body=None,
    omega=None,
    alpha=None,
    gravity=(0, 0, -9.81),
):
    """Robot-on-body wrench needed for measured rigid-body motion, in world axes."""
    com_pos = np.asarray(com_pos, float)
    acc = np.zeros_like(com_pos) if com_acc is None else np.asarray(com_acc, float)
    f = mass * (acc - np.asarray(gravity))
    tau = np.cross(com_pos - np.asarray(reference_pos), f)
    if inertia_body is not None:
        r = np.asarray(rotation_world_body)
        iw = r @ np.asarray(inertia_body) @ np.swapaxes(r, -1, -2)
        omega, alpha = np.asarray(omega), np.asarray(alpha)
        tau += np.einsum("...ij,...j->...i", iw, alpha) + np.cross(
            omega, np.einsum("...ij,...j->...i", iw, omega)
        )
    return np.concatenate([f, tau], axis=-1)


def derivative(values, time):
    values, time = np.asarray(values, float), np.asarray(time, float)
    if len(time) < 3 or np.any(np.diff(time) <= 0):
        raise ValueError("need at least three strictly increasing timestamps")
    return np.gradient(values, time, axis=0, edge_order=2)


def quaternion_angle(a, b):
    a, b = np.asarray(a), np.asarray(b)
    dot = np.sum(a * b, axis=-1) / (
        np.linalg.norm(a, axis=-1) * np.linalg.norm(b, axis=-1)
    )
    return 2 * np.arccos(np.clip(np.abs(dot), 0, 1))


def interpolate_quaternions(time, quaternions, query):
    """Shortest-arc SLERP, insensitive to equivalent q/-q sign changes."""
    time, query = np.asarray(time), np.asarray(query)
    q = np.asarray(quaternions, float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    i = np.clip(np.searchsorted(time, query, side="right") - 1, 0, len(time) - 2)
    u = ((query - time[i]) / (time[i + 1] - time[i]))[:, None]
    a, b = q[i], q[i + 1].copy()
    dot = np.sum(a * b, axis=1, keepdims=True)
    b = np.where(dot < 0, -b, b)
    angle = np.arccos(np.clip(np.abs(dot), 0, 1))
    # sinc form avoids the removable singularity when the two poses coincide.
    result = (
        (1 - u) * np.sinc((1 - u) * angle / np.pi) * a
        + u * np.sinc(u * angle / np.pi) * b
    ) / np.sinc(angle / np.pi)
    return result / np.linalg.norm(result, axis=1, keepdims=True)
