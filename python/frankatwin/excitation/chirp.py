#!/usr/bin/env python3
"""Generate the chirp excitation trajectory (Franka).

A linear frequency sweep, where frankatwin.excitation.multiband holds a few
stationary tones per axis. The design follows UR5e's collect_sysid_data.py
(linear chirp, 6-DOF, evenly phase-staggered) but with bounds tuned for the
Franka task-impedance loop.

What the sweep buys over stationary tones:

  * Frequency profile: a linear chirp f0->f1 instead of fixed bands per axis.
    It visits every frequency between f0 and f1 once, giving CMA-ES a much
    denser spectral footprint to fit armature / viscous / delay against.

  * All 6 Cartesian DOFs active simultaneously (x, y, z, rx, ry, rz) with
    phase offsets at k * pi/3, so J5/J6/J7 friction is identifiable.

  * Z amplitude > XY amplitude.  Vertical motion couples into J2/J3 gravity
    terms, so giving Z more energy improves identification of shoulder/elbow
    dynamics.

  * Asymmetric linear ramp (2 s up / 3 s down) matching UR5e's convention.

Two bands, 8 s each (``CHIRP_BANDS``); ``low`` is the fit run, ``high`` an
optional held-out run:

  * ``low``  -- 0.1 -> 0.7 Hz at constant amplitude: the fit run
    (``amp_taper_exp = 0``).
  * ``high`` -- 0.7 -> 3.0 Hz with the amplitude tapered as (f0 / f)^0.5
    (``amp_taper_exp = 0.5``) and a 1 s fade-out instead of 3 s, so the upper
    third of the sweep runs at full envelope. A constant-amplitude sweep
    (UR5e / OmniReset: 0.1 -> 3 Hz in one 8 s chirp) is what made f1 drop to
    0.7 Hz here. The band is sized for the fit gains kp 200/20, from a run on
    the arm (2026-09-21; exponent 1, 3 s fade-out):

      - The arm followed 0.44 / 0.20 / 0.11 of the z reference at
        1.4 / 2.0 / 2.6 Hz. A 1-DOF model of the impedance loop (kd =
        2 sqrt(kp), 50 Hz ZOH) reproduces that roll-off with an effective mass
        of ~7.5 kg, i.e. a loop bandwidth of ~0.8 Hz at kp 200: the whole band
        lies above it, the motion falls as reference / f^2, and the load is
        set by the loop, not by the reference acceleration.
      - That left 34 / 11 / 2.8 mm of z motion around 2.5 / 4.5 / 6.5 s and
        0.6-5 mrad per joint in the last window -- too little to constrain
        anything -- at a wrist load of 13 / 23 / 5 % of the 12 N*m limit
        (``tau_J_limit_frac``, gravity included).
      - For exponent 0.5 with the 1 s fade-out the calibrated model predicts
        ~60 / 19 / 10 mm at ~33 N of peak EE force (1.4x that run's) and
        ~0.56 m/s. A prediction: this default has not run on the arm yet.

    Exponent 0 (constant amplitude) would move the arm more still, but the
    torque step of each 50 Hz setpoint grows with the reference, osc_shm ramps
    any step above 0.8 N*m per tick (its 800 N*m/s slew limit), and the sim
    models neither that limit nor the 87/12 N*m clamp (it clamps at 100).
    Estimated steps: ~0.8 N*m at exponent 1, ~1.6 at 0.5 (a 2 ms ramp), ~3.1
    at 0 (4 ms -- the whole 0-4 ms range the delay is searched in). At kp
    500/30 the arm follows further up the band and the model puts the default
    at ~84 N: use ``--amp-taper-exp 2`` there (~30 N).

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
# for the Franka task-impedance envelope.  Peak target rates are reported by
# _peak_rates for inspection; nothing caps them.
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
# Sweep bands: keyword arguments of build_chirp_trajectory (see the module
# docstring). `amp_taper_exp` p scales the amplitude by (f0 / f_inst)^p along
# the sweep: 0 = constant amplitude, 1 = constant reference velocity,
# 2 = constant reference acceleration. The high band's 0.5 and its 1 s
# fade-out are sized for kp 200/20 from a run on the arm.
CHIRP_BANDS = {
    "low": {"f0": CHIRP_F0_DEFAULT, "f1": CHIRP_F1_DEFAULT, "amp_taper_exp": 0.0,
            "ramp_up_s": 2.0, "ramp_down_s": 3.0},
    "high": {"f0": 0.7, "f1": 3.0, "amp_taper_exp": 0.5,
             "ramp_up_s": 2.0, "ramp_down_s": 1.0},
}
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


def _amp_taper(t_s: np.ndarray, f0: float, f1: float, total_T: float, p: float) -> tuple[np.ndarray, np.ndarray]:
    """Amplitude envelope a(t) = (f0 / f_inst(t))^p and its derivative.

    p = 0 leaves the amplitude constant. p = 1 keeps A * 2 pi f_inst -- the
    reference velocity -- at its f0 value for the whole sweep, p = 2 does the
    same for the reference acceleration A * (2 pi f_inst)^2.
    """
    if p == 0.0 or total_T <= 0.0:
        return np.ones_like(t_s), np.zeros_like(t_s)
    if f0 <= 0.0:
        raise ValueError("amp_taper_exp > 0 needs f0 > 0")
    sweep_rate = (f1 - f0) / total_T
    f_inst = f0 + sweep_rate * t_s
    a = (f0 / f_inst) ** p
    a_dot = -p * a * sweep_rate / f_inst
    return a, a_dot


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
    amp_taper_exp: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build a 6-DOF chirp reference trajectory around the given anchor pose.

    ``amp_taper_exp`` scales every amplitude by (f0 / f_inst)^p along the
    sweep (0 = constant amplitude, the low band; 0.5, the high band;
    1 = constant reference velocity; 2 = constant reference acceleration --
    see ``CHIRP_BANDS``).

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
    taper, taper_dot = _amp_taper(t_s, f0, f1, total_T, float(amp_taper_exp))
    env = rho * taper
    env_dot = rho_dot * taper + rho * taper_dot

    amps = np.array([amp_x, amp_y, amp_z, amp_rx, amp_ry, amp_rz], dtype=np.float64)
    pos_offsets = np.zeros((t_s.shape[0], 3), dtype=np.float64)
    dpos_offsets = np.zeros((t_s.shape[0], 3), dtype=np.float64)
    rot_offsets = np.zeros((t_s.shape[0], 3), dtype=np.float64)

    for i in range(6):
        s = np.sin(phi + PHASE_OFFSETS[i])
        c = np.cos(phi + PHASE_OFFSETS[i])
        val = amps[i] * env * s
        dval = amps[i] * (env_dot * s + env * phi_dot * c)
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

