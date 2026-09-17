#!/usr/bin/env python3
"""Generate the SysID v4 chirp excitation trajectory (Franka).

v4 supersedes v3 (multiband, see frankatwin.excitation.multiband) by switching from a
two-band stationary sinusoid to a linear frequency sweep ("chirp"). The
design follows UR5e's collect_sysid_data.py (linear chirp, 6-DOF, evenly
phase-staggered) but with bounds tuned for the Franka task-impedance loop.

Key differences vs v3 (multiband):

  * Frequency profile: linear chirp f0->f1 instead of two fixed bands per axis.
    This sweeps every frequency between f0 and f1 once, giving CMA-ES a much
    denser spectral footprint to fit armature / viscous / delay against.

  * All 6 Cartesian DOFs active simultaneously (x, y, z, rx, ry, rz) with
    phase offsets at k * pi/3.  v3 left rotation off by default; v4 always
    excites rotation so J5/J6/J7 friction is identifiable.

  * Z amplitude > XY amplitude (v3 had Z smallest).  Vertical motion couples
    into J2/J3 gravity terms, so giving Z more energy improves identification
    of shoulder/elbow dynamics.

  * Asymmetric linear ramp (2 s up / 3 s down) matching UR5e's convention.

The sidecar JSON written by this script is compatible with apply_sysid_params
(``--invoke-replay``) and replay_python_csv_sim.py for downstream sim/real
comparison.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# Defaults follow the UR5e chirp template (collect_sysid_data.py) but scaled
# for the Franka task-impedance envelope.  The sidecar peak-rate conventions
# (peak Cartesian speed <= 0.30 m/s, peak angular rate <= 0.50 rad/s) still
# apply -- see _print_peak_rates for the runtime warnings.
# ---------------------------------------------------------------------------

# Defaults follow UR5e's collect_sysid_data.py *shape* exactly (amplitudes,
# Z=1.5xXY ratio, asymmetric 2s/3s ramp, pi/3 phase offsets, 8 s duration)
# but with f1 lowered from UR5e's 3.0 Hz to the Franka production value of
# 0.7 Hz.  This keeps peak |dx| well under 0.5 m/s (vs ~2 m/s at 3.0 Hz / ~1
# m/s at the earlier 1.5 Hz interim value) so the Franka task-impedance loop
# can actually track it, J5-J7 effort stays clear of the 12 N*m saturation
# clamp, and the 50 Hz cart_impedance.py logger is well above Nyquist.  The
# per-axis spectral *shape* (linear chirp from 0.1 Hz to f1) is preserved.
#
# NOTE: With UR5e-amp rotations (0.50 + 0.25 + 0.50 rad), the reference itself
# reaches max|rot_offset| ~= 0.61 rad.  The orientation channel is pure
# impedance -- no clamp, no tracking abort -- so nothing aborts on that; the
# arm simply lags the reference by however much the gains allow.
CHIRP_F0_DEFAULT = 0.1
CHIRP_F1_DEFAULT = 0.7
# Per-axis phase offsets (6 axes, 6 evenly spaced offsets k * pi/3).
PHASE_OFFSETS = np.array([0.0, np.pi / 3.0, 2.0 * np.pi / 3.0,
                          np.pi, 4.0 * np.pi / 3.0, 5.0 * np.pi / 3.0])


def _load_base_sidecar(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    for key in ("q_init", "x_anchor", "q_anchor_xyzw", "args"):
        if key not in payload:
            raise KeyError(f"{path} missing key: {key}")
    return payload


def _linear_ramp(t_s: np.ndarray, total_T: float, ramp_up_s: float, ramp_down_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Asymmetric linear ramp envelope rho(t) in [0,1] with derivative rho_dot.

    rho(0) = 0, rho(ramp_up_s) = 1, rho(T - ramp_down_s) = 1, rho(T) = 0.
    rho is piecewise linear: rising slope = 1/ramp_up_s, flat at 1, falling
    slope = -1/ramp_down_s.  rho_dot is the corresponding slope (0 in flat
    region) -- not C1 at the corners, but UR5e's chirp uses the same shape
    and the controller smooths the resulting target with its inertia + Kd.
    """
    rho = np.ones_like(t_s)
    rho_dot = np.zeros_like(t_s)
    if ramp_up_s > 0.0:
        in_up = t_s < ramp_up_s
        rho[in_up] = t_s[in_up] / ramp_up_s
        rho_dot[in_up] = 1.0 / ramp_up_s
    if ramp_down_s > 0.0 and total_T > ramp_down_s:
        t_down_start = total_T - ramp_down_s
        in_down = t_s >= t_down_start
        rho[in_down] = np.clip((total_T - t_s[in_down]) / ramp_down_s, 0.0, 1.0)
        rho_dot[in_down] = -1.0 / ramp_down_s
    rho = np.clip(rho, 0.0, 1.0)
    return rho, rho_dot


def _chirp_phase(t_s: np.ndarray, f0: float, f1: float, total_T: float) -> tuple[np.ndarray, np.ndarray]:
    """Linear chirp instantaneous phase and angular frequency.

    phi(t)   = 2*pi*(f0*t + 0.5*(f1-f0)/T * t^2)
    phi'(t)  = 2*pi*(f0 + (f1-f0)/T * t)
    """
    if total_T <= 0.0:
        return np.zeros_like(t_s), np.zeros_like(t_s)
    sweep_rate = (f1 - f0) / total_T  # Hz / s
    phi = 2.0 * np.pi * (f0 * t_s + 0.5 * sweep_rate * t_s ** 2)
    phi_dot = 2.0 * np.pi * (f0 + sweep_rate * t_s)
    return phi, phi_dot


def _axis_angle_to_quat_xyzw(rot_vec: np.ndarray) -> np.ndarray:
    """(T,3) axis-angle -> (T,4) unit quaternion in xyzw.  Identity at theta=0."""
    theta = np.linalg.norm(rot_vec, axis=1)
    eps = 1e-9
    safe_theta = np.where(theta < eps, 1.0, theta)
    axis = rot_vec / safe_theta[:, None]
    half = 0.5 * theta
    sin_half = np.sin(half)
    cos_half = np.cos(half)
    # Force identity for near-zero rotations.
    sin_half = np.where(theta < eps, 0.0, sin_half)
    cos_half = np.where(theta < eps, 1.0, cos_half)
    q = np.zeros((rot_vec.shape[0], 4), dtype=np.float64)
    q[:, 0] = axis[:, 0] * sin_half
    q[:, 1] = axis[:, 1] * sin_half
    q[:, 2] = axis[:, 2] * sin_half
    q[:, 3] = cos_half
    return q


def _quat_mul_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two (...,4) xyzw quaternion arrays."""
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    w = aw * bw - ax * bx - ay * by - az * bz
    return np.stack((x, y, z, w), axis=-1)


def build_chirp_trajectory(
    t_s: np.ndarray,
    x_anchor: np.ndarray,
    q_anchor_xyzw: np.ndarray,
    *,
    f0: float = CHIRP_F0_DEFAULT,
    f1: float = CHIRP_F1_DEFAULT,
    amp_x: float = 0.10,
    amp_y: float = 0.10,
    amp_z: float = 0.15,
    amp_rx: float = 0.50,
    amp_ry: float = 0.25,
    amp_rz: float = 0.50,
    ramp_up_s: float = 2.0,
    ramp_down_s: float = 3.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build a 6-DOF chirp reference trajectory around the given anchor pose.

    Returns:
        ``(x_des, dx_des, quat_des_xyzw, rot_offsets)`` --
        ``x_des`` (T, 3) Cartesian target positions;
        ``dx_des`` (T, 3) analytic target velocities;
        ``quat_des_xyzw`` (T, 4) target orientation (xyzw), the anchor
        pre-rotated by the world-frame axis-angle offset;
        ``rot_offsets`` (T, 3) that axis-angle offset (for diagnostics and
        the sidecar).
    """
    t_s = np.asarray(t_s, dtype=np.float64)
    x_anchor = np.asarray(x_anchor, dtype=np.float64).reshape(3)
    q_anchor_xyzw = np.asarray(q_anchor_xyzw, dtype=np.float64).reshape(4)
    total_T = float(t_s[-1]) if t_s.size > 0 else 0.0

    phi, phi_dot = _chirp_phase(t_s, f0, f1, total_T)
    rho, rho_dot = _linear_ramp(t_s, total_T, ramp_up_s, ramp_down_s)

    amps = np.array([amp_x, amp_y, amp_z, amp_rx, amp_ry, amp_rz], dtype=np.float64)
    pos_offsets = np.zeros((t_s.shape[0], 3), dtype=np.float64)
    dpos_offsets = np.zeros((t_s.shape[0], 3), dtype=np.float64)
    rot_offsets = np.zeros((t_s.shape[0], 3), dtype=np.float64)

    for i in range(6):
        s = np.sin(phi + PHASE_OFFSETS[i])
        c = np.cos(phi + PHASE_OFFSETS[i])
        val = amps[i] * rho * s
        dval = amps[i] * (rho_dot * s + rho * phi_dot * c)
        if i < 3:
            pos_offsets[:, i] = val
            dpos_offsets[:, i] = dval
        else:
            rot_offsets[:, i - 3] = val
            # Rotation velocity is not returned (matches UR5e -- target_quat
            # alone is enough for the OSC, and analytic angular velocity is
            # noisy near identity).

    x_des = x_anchor[None, :] + pos_offsets
    dx_des = dpos_offsets

    # Compose target quaternion: q_des = q_offset(world) (x) q_anchor
    q_offset = _axis_angle_to_quat_xyzw(rot_offsets)
    q_anchor_tiled = np.broadcast_to(q_anchor_xyzw[None, :], (t_s.shape[0], 4)).copy()
    quat_des = _quat_mul_xyzw(q_offset, q_anchor_tiled)
    quat_des = quat_des / np.clip(np.linalg.norm(quat_des, axis=1, keepdims=True), 1e-12, None)

    return x_des, dx_des, quat_des, rot_offsets


def _peak_rates(t_s: np.ndarray, dx_des: np.ndarray, rot_offsets: np.ndarray) -> dict:
    """Peak Cartesian speed + rotation rates, computed for safety pre-flight."""
    if t_s.size > 1:
        dt = float(t_s[1] - t_s[0])
    else:
        dt = 0.001
    peak_dx = float(np.max(np.abs(dx_des[:, 0])))
    peak_dy = float(np.max(np.abs(dx_des[:, 1])))
    peak_dz = float(np.max(np.abs(dx_des[:, 2])))
    peak_cart_speed = float(np.max(np.linalg.norm(dx_des, axis=1)))
    # Rotation rate: finite-difference of each axis-angle component.  Cross-axis
    # angular velocity coupling is small at these amplitudes (~0.4 rad), so
    # per-axis dRx/dt etc. is a fine proxy.
    drot = np.gradient(rot_offsets, dt, axis=0)
    peak_drx = float(np.max(np.abs(drot[:, 0])))
    peak_dry = float(np.max(np.abs(drot[:, 1])))
    peak_drz = float(np.max(np.abs(drot[:, 2])))
    peak_ang_rate = float(np.max(np.linalg.norm(drot, axis=1)))
    return {
        "cart_speed_m_s": peak_cart_speed,
        "dx_m_s": [peak_dx, peak_dy, peak_dz],
        "ang_rate_rad_s": peak_ang_rate,
        "drot_rad_s": [peak_drx, peak_dry, peak_drz],
    }


# Safety conventions (shared with frankatwin.excitation.multiband).
CART_DX_PEAK_LIMIT_MPS = 0.30
ORI_DOT_PEAK_LIMIT_RPS = 0.50
