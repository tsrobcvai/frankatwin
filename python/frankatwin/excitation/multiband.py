#!/usr/bin/env python3
"""Generate the SysID v3 multi-band excitation trajectory (Franka).

Two-band stationary sinusoid per Cartesian axis (~0.15-0.30 Hz low band plus a
0.7-1.1 Hz high band at ``--high-band-ratio`` amplitude), plus two optional
multi-band rotation sweeps: one about base-z (drives j1 directly) and one
about EE-z (drives j5/j7).

Position-only excitation (``--amp-yaw 0 --amp-roll 0``, the v1/v2 variant)
leaves j1 (base yaw) and j5 (wrist roll) under-excited because the task wrench
has no torque component; v3 adds the rotation sweeps to fix that.

Defaults reproduce the v3 collection the published sysid parameters were
fitted on: amp 0.10 / 0.10 / 0.08 m, yaw 0.25 rad, roll 0.20 rad,
high_band_ratio 0.20, amp_ramp 2.0 s, duration 12 s.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def _load_base_sidecar(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    for key in ("q_init", "x_anchor", "q_anchor_xyzw", "args"):
        if key not in payload:
            raise KeyError(f"{path} missing key: {key}")
    return payload


def _half_cosine_envelope(t_s: np.ndarray, ramp_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Return (rho, rho_dot) where rho is a smooth 0->1 envelope.

    rho(t)  = 0.5 * (1 - cos(pi * t / T))   for 0 <= t <= T,  else 1
    rho'(t) = 0.5 * (pi / T) * sin(pi * t / T) for 0 <= t <= T, else 0

    Boundary conditions: rho(0) = rho'(0) = 0 and rho(T) = 1, rho'(T) = 0,
    so the resulting trajectory starts exactly at the anchor with zero
    velocity / zero acceleration jump, and ramp-out is smooth too.
    """
    if ramp_s <= 0.0:
        return np.ones_like(t_s), np.zeros_like(t_s)
    s = np.clip(t_s / ramp_s, 0.0, 1.0)
    rho = 0.5 * (1.0 - np.cos(np.pi * s))
    in_ramp = (t_s >= 0.0) & (t_s < ramp_s)
    rho_dot = np.zeros_like(t_s)
    rho_dot[in_ramp] = 0.5 * (np.pi / ramp_s) * np.sin(np.pi * s[in_ramp])
    return rho, rho_dot


POS_FREQS = {
    "x": (0.15, 0.70, 0.0),                # (low, high, phase)
    "y": (0.20, 0.90, np.pi / 3.0),
    "z": (0.30, 1.10, np.pi / 4.0),
}

# Rotation bands sit between position bands to give CMA-ES a clean
# spectral fingerprint for each DOF (no aliasing with translation harmonics).
ORI_FREQS = {
    "yaw":  (0.18, 0.55, 0.0),
    "roll": (0.22, 0.65, np.pi / 5.0),
}


def _two_band(t_s: np.ndarray, freq_lo: float, freq_hi: float, phase_lo: float, ratio: float) -> tuple[np.ndarray, np.ndarray]:
    """Return (value, derivative) of ``sin(2π·f_lo·t + φ) + ratio·sin(2π·f_hi·t)``."""
    two_pi = 2.0 * np.pi
    val = np.sin(two_pi * freq_lo * t_s + phase_lo) + ratio * np.sin(two_pi * freq_hi * t_s)
    dval = (
        two_pi * freq_lo * np.cos(two_pi * freq_lo * t_s + phase_lo)
        + ratio * two_pi * freq_hi * np.cos(two_pi * freq_hi * t_s)
    )
    return val, dval


def _build_pos_traj(
    t_s: np.ndarray,
    x_anchor: np.ndarray,
    amp_x: float,
    amp_y: float,
    amp_z: float,
    rho: np.ndarray,
    rho_dot: np.ndarray,
    high_band_ratio: float,
) -> tuple[np.ndarray, np.ndarray]:
    bx, dbx = _two_band(t_s, POS_FREQS["x"][0], POS_FREQS["x"][1], POS_FREQS["x"][2], high_band_ratio)
    by, dby = _two_band(t_s, POS_FREQS["y"][0], POS_FREQS["y"][1], POS_FREQS["y"][2], high_band_ratio)
    bz, dbz = _two_band(t_s, POS_FREQS["z"][0], POS_FREQS["z"][1], POS_FREQS["z"][2], high_band_ratio)
    bx, by, bz = amp_x * bx, amp_y * by, amp_z * bz
    dbx, dby, dbz = amp_x * dbx, amp_y * dby, amp_z * dbz

    x = x_anchor[0] + rho * bx
    y = x_anchor[1] + rho * by
    z = x_anchor[2] + rho * bz
    dx = rho_dot * bx + rho * dbx
    dy = rho_dot * by + rho * dby
    dz = rho_dot * bz + rho * dbz
    return np.column_stack((x, y, z)), np.column_stack((dx, dy, dz))


def _quat_mul_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two (...,4) xyzw quaternion arrays."""
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    w = aw * bw - ax * bx - ay * by - az * bz
    return np.stack((x, y, z, w), axis=-1)


def _quat_from_axis_angle_z(angle: np.ndarray) -> np.ndarray:
    """Return (T,4) xyzw quaternion array for rotation about local-z by ``angle`` [rad]."""
    half = 0.5 * angle
    out = np.zeros((angle.shape[0], 4), dtype=np.float64)
    out[:, 2] = np.sin(half)
    out[:, 3] = np.cos(half)
    return out


def _build_quat_traj(
    t_s: np.ndarray,
    q_anchor_xyzw: np.ndarray,
    amp_yaw: float,
    amp_roll: float,
    rho: np.ndarray,
    high_band_ratio: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compose q_des(t) = q_yaw_base(t) ⊗ q_anchor ⊗ q_roll_local(t).

    * yaw : rotation about world (base) z-axis, multi-band, drives j1.
    * roll: rotation about EE local z-axis, multi-band, drives j5/j7.
    Both default to amp=0, in which case q_des(t) = q_anchor (position-only variant).
    """
    yaw_freqs = ORI_FREQS["yaw"]
    roll_freqs = ORI_FREQS["roll"]

    if amp_yaw > 0.0:
        yaw_unit, _ = _two_band(t_s, yaw_freqs[0], yaw_freqs[1], yaw_freqs[2], high_band_ratio)
        yaw = rho * amp_yaw * yaw_unit
    else:
        yaw = np.zeros_like(t_s)
    if amp_roll > 0.0:
        roll_unit, _ = _two_band(t_s, roll_freqs[0], roll_freqs[1], roll_freqs[2], high_band_ratio)
        roll = rho * amp_roll * roll_unit
    else:
        roll = np.zeros_like(t_s)

    q_yaw_base = _quat_from_axis_angle_z(yaw)
    q_roll_local = _quat_from_axis_angle_z(roll)
    q_anchor_tiled = np.broadcast_to(q_anchor_xyzw[None, :], (t_s.shape[0], 4))
    q_des = _quat_mul_xyzw(q_yaw_base, _quat_mul_xyzw(q_anchor_tiled, q_roll_local))
    q_des = q_des / np.clip(np.linalg.norm(q_des, axis=1, keepdims=True), 1e-12, None)
    return q_des, yaw, roll


def build_multiband_trajectory(
    t_s: np.ndarray,
    x_anchor: np.ndarray,
    q_anchor_xyzw: np.ndarray,
    amp_x: float = 0.10,
    amp_y: float = 0.10,
    amp_z: float = 0.08,
    amp_yaw: float = 0.25,
    amp_roll: float = 0.20,
    high_band_ratio: float = 0.20,
    amp_ramp_s: float = 2.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build the v3 multi-band excitation trajectory on an arbitrary time grid.

    Returns
    -------
    x_des : (N, 3) Cartesian target positions, in the same frame as ``x_anchor``.
    dx_des : (N, 3) target Cartesian velocities (analytic derivative).
    quat_des : (N, 4) target orientation quaternions, **xyzw** order, unit-norm.
    yaw : (N,) yaw angle (about world-z) applied to the anchor.
    roll : (N,) roll angle (about EE local-z) applied to the anchor.

    Setting both ``amp_yaw`` and ``amp_roll`` to 0 gives the position-only
    variant (orientation held at ``q_anchor_xyzw``). Default amplitudes match
    the v3 collection (0.10/0.10/0.08 m, yaw 0.25, roll 0.20, high_band_ratio
    0.20).
    """
    rho, rho_dot = _half_cosine_envelope(t_s, amp_ramp_s)
    x_des, dx_des = _build_pos_traj(t_s, x_anchor, amp_x, amp_y, amp_z, rho, rho_dot, high_band_ratio)
    quat_des, yaw, roll = _build_quat_traj(t_s, q_anchor_xyzw, amp_yaw, amp_roll, rho, high_band_ratio)
    return x_des, dx_des, quat_des, yaw, roll
