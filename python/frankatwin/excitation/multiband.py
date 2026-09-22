#!/usr/bin/env python3
"""Generate the multi-band excitation trajectory (Franka).

Stationary sinusoids ("tones") summed per Cartesian axis: unlike the chirp
(frankatwin.excitation.chirp), every frequency is present for the whole run.
One profile ships (``MULTIBAND_PROFILES["heldout"]``), the held-out validation
run for parameters fitted on the chirps: same DOFs and frequency range as the
fit data, different waveform.

  * Tilt. Rotations about world x and y (``amp_rx`` / ``amp_ry``) and about
    base z ("yaw", ``amp_yaw``) are excited, so all three rotational DOFs
    move. Roll about EE z (``amp_roll``) is off: with the tool pointing
    straight down -- the home pose -- EE-z is anti-parallel to base-z, so yaw
    and roll turn about the same vertical line and roll would only duplicate
    yaw. Translation alone would leave j1 (base yaw) and j5 (wrist roll)
    under-excited, because the task wrench has no torque component.
  * Spectrum. Three tones per axis (``HELDOUT_FREQS``) at 0.15-0.30, 0.55-1.1
    and 1.25-2.0 Hz, all 21 distinct. A tone at frequency f has relative
    amplitude min(1, 0.35 / f) (``HELDOUT_CORNER_HZ``), i.e. every tone above
    0.35 Hz has the same peak reference velocity (a corner / f taper; the
    chirp high band uses (f0 / f)^0.5). The upper two tones carry 12-33 % of
    the reference's position power, depending on the axis.
  * Envelope. 2 s half-cosine fade-in and fade-out, so the target starts and
    ends at the anchor. 12 s.

Defaults: amp 0.06 m on x / y / z, yaw 0.25 rad, tilt 0.20 / 0.15 rad.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .chirp import _axis_angle_to_quat_xyzw


def _load_base_sidecar(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    for key in ("q_init", "x_anchor", "q_anchor_xyzw", "args"):
        if key not in payload:
            raise KeyError(f"{path} missing key: {key}")
    return payload


def _half_cosine_envelope(t_s: np.ndarray, ramp_s: float, ramp_down_s: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Return (rho, rho_dot) where rho is a smooth 0->1(->0) envelope.

    rho(t)  = 0.5 * (1 - cos(pi * t / T))   for 0 <= t <= T,  else 1
    rho'(t) = 0.5 * (pi / T) * sin(pi * t / T) for 0 <= t <= T, else 0

    Boundary conditions: rho(0) = rho'(0) = 0 and rho(T) = 1, rho'(T) = 0,
    so the resulting trajectory starts exactly at the anchor with zero
    velocity / zero acceleration jump.

    ``ramp_down_s > 0`` multiplies in the mirrored fade-out over the last
    ``ramp_down_s`` seconds of the grid (it ends at ``t_s[-1]``, as the chirp's
    ramp does), so the trajectory also returns to the anchor. 0 leaves the
    envelope at 1.
    """
    if ramp_s <= 0.0:
        rho, rho_dot = np.ones_like(t_s), np.zeros_like(t_s)
    else:
        s = np.clip(t_s / ramp_s, 0.0, 1.0)
        rho = 0.5 * (1.0 - np.cos(np.pi * s))
        in_ramp = (t_s >= 0.0) & (t_s < ramp_s)
        rho_dot = np.zeros_like(t_s)
        rho_dot[in_ramp] = 0.5 * (np.pi / ramp_s) * np.sin(np.pi * s[in_ramp])
    if ramp_down_s <= 0.0 or t_s.size == 0:
        return rho, rho_dot
    u = np.clip((t_s[-1] - t_s) / ramp_down_s, 0.0, 1.0)
    down = 0.5 * (1.0 - np.cos(np.pi * u))
    down_dot = np.zeros_like(t_s)
    in_down = u < 1.0
    down_dot[in_down] = -0.5 * (np.pi / ramp_down_s) * np.sin(np.pi * u[in_down])
    return rho * down, rho_dot * down + rho * down_dot


# Tone table, three tones per axis: (low, mid, high) [Hz]. Rotation bands sit
# between position bands to give CMA-ES a clean spectral fingerprint for each
# DOF (no aliasing with translation harmonics); all 21 are distinct.
HELDOUT_FREQS = {
    "x":    (0.15, 0.70, 1.40),
    "y":    (0.20, 0.90, 1.70),
    "z":    (0.30, 1.10, 2.00),
    "rx":   (0.24, 0.75, 1.60),
    "ry":   (0.26, 0.80, 1.85),
    "yaw":  (0.18, 0.55, 1.25),
    "roll": (0.22, 0.65, 1.50),
}
# Relative amplitude of a heldout tone: min(1, HELDOUT_CORNER_HZ / f), constant
# peak reference velocity above the corner.
HELDOUT_CORNER_HZ = 0.35

AXES = ("x", "y", "z", "rx", "ry", "yaw", "roll")


def multiband_tones(profile: str = "heldout") -> dict:
    """Return the tone table of ``profile``: ``{axis: ((freq_hz, rel_amp, phase_rad), ...)}``.

    ``heldout``: three tones per axis at min(1, HELDOUT_CORNER_HZ / f), phases
    staggered by pi/3 per axis and pi/2 per band.
    """
    if profile == "heldout":
        return {
            axis: tuple(
                (f, min(1.0, HELDOUT_CORNER_HZ / f), (i * np.pi / 3.0 + band * np.pi / 2.0) % (2.0 * np.pi))
                for band, f in enumerate(HELDOUT_FREQS[axis])
            )
            for i, axis in enumerate(AXES)
        }
    raise ValueError(f"unknown multiband profile: {profile!r} (choose from {sorted(MULTIBAND_PROFILES)})")


# Keyword arguments of build_multiband_trajectory per profile (see the module
# docstring). 12 s.
MULTIBAND_PROFILES = {
    "heldout": {
        "amp_x": 0.06, "amp_y": 0.06, "amp_z": 0.06,
        "amp_yaw": 0.25, "amp_roll": 0.0, "amp_rx": 0.20, "amp_ry": 0.15,
        "amp_ramp_s": 2.0, "ramp_down_s": 2.0, "profile": "heldout",
    },
}


def _multi_tone(t_s: np.ndarray, tones) -> tuple[np.ndarray, np.ndarray]:
    """Return (value, derivative) of ``sum_k a_k·sin(2π·f_k·t + φ_k)`` over ``tones`` = ((f_k, a_k, φ_k), ...)."""
    two_pi = 2.0 * np.pi
    val = np.zeros_like(t_s)
    dval = np.zeros_like(t_s)
    for freq, rel_amp, phase in tones:
        val = val + rel_amp * np.sin(two_pi * freq * t_s + phase)
        dval = dval + rel_amp * two_pi * freq * np.cos(two_pi * freq * t_s + phase)
    return val, dval


def _build_pos_traj(
    t_s: np.ndarray,
    x_anchor: np.ndarray,
    amp_x: float,
    amp_y: float,
    amp_z: float,
    rho: np.ndarray,
    rho_dot: np.ndarray,
    tones: dict,
) -> tuple[np.ndarray, np.ndarray]:
    bx, dbx = _multi_tone(t_s, tones["x"])
    by, dby = _multi_tone(t_s, tones["y"])
    bz, dbz = _multi_tone(t_s, tones["z"])
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
    amp_rx: float,
    amp_ry: float,
    rho: np.ndarray,
    tones: dict,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compose q_des(t) = q_yaw_base(t) ⊗ q_tilt_base(t) ⊗ q_anchor ⊗ q_roll_local(t).

    * yaw : rotation about world (base) z-axis.
    * tilt: rotation by the world-frame axis-angle vector (rx, ry, 0).
    * roll: rotation about EE local z-axis.
    An amplitude of 0 switches its rotation off; with all four at 0,
    q_des(t) = q_anchor (position-only variant). At a tool-down anchor yaw and
    roll are the same rotation (see the module docstring); tilt is what moves
    the tool axis.
    """
    def _angle(amp: float, axis: str) -> np.ndarray:
        if amp > 0.0:
            unit, _ = _multi_tone(t_s, tones[axis])
            return rho * amp * unit
        return np.zeros_like(t_s)

    yaw = _angle(amp_yaw, "yaw")
    roll = _angle(amp_roll, "roll")
    tilt = np.column_stack((_angle(amp_rx, "rx"), _angle(amp_ry, "ry")))

    q_yaw_base = _quat_from_axis_angle_z(yaw)
    q_roll_local = _quat_from_axis_angle_z(roll)
    q_anchor_tiled = np.broadcast_to(q_anchor_xyzw[None, :], (t_s.shape[0], 4))
    q_des = _quat_mul_xyzw(q_anchor_tiled, q_roll_local)
    if amp_rx > 0.0 or amp_ry > 0.0:
        q_tilt_base = _axis_angle_to_quat_xyzw(np.column_stack((tilt, np.zeros_like(t_s))))
        q_des = _quat_mul_xyzw(q_tilt_base, q_des)
    q_des = _quat_mul_xyzw(q_yaw_base, q_des)
    q_des = q_des / np.clip(np.linalg.norm(q_des, axis=1, keepdims=True), 1e-12, None)
    return q_des, yaw, roll, tilt


def build_multiband_trajectory(
    t_s: np.ndarray,
    x_anchor: np.ndarray,
    q_anchor_xyzw: np.ndarray,
    amp_x: float = 0.06,
    amp_y: float = 0.06,
    amp_z: float = 0.06,
    amp_yaw: float = 0.25,
    amp_roll: float = 0.0,
    amp_ramp_s: float = 2.0,
    *,
    amp_rx: float = 0.20,
    amp_ry: float = 0.15,
    ramp_down_s: float = 2.0,
    profile: str = "heldout",
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build the multi-band excitation trajectory on an arbitrary time grid.

    Returns
    -------
    x_des : (N, 3) Cartesian target positions, in the same frame as ``x_anchor``.
    dx_des : (N, 3) target Cartesian velocities (analytic derivative).
    quat_des : (N, 4) target orientation quaternions, **xyzw** order, unit-norm.
    yaw : (N,) yaw angle (about world-z) applied to the anchor.
    roll : (N,) roll angle (about EE local-z) applied to the anchor.
    tilt : (N, 2) world-frame axis-angle components (rx, ry) applied to the anchor.

    The defaults are ``MULTIBAND_PROFILES["heldout"]`` (0.06 m on x / y / z,
    yaw 0.25 rad, tilt 0.20 / 0.15 rad, roll off, 2 s fade-in and fade-out).
    ``profile`` picks the tone table (``multiband_tones``) and nothing else.
    Setting the four rotation amplitudes to 0 gives the position-only variant
    (orientation held at ``q_anchor_xyzw``).
    """
    tones = multiband_tones(profile)
    rho, rho_dot = _half_cosine_envelope(t_s, amp_ramp_s, ramp_down_s)
    x_des, dx_des = _build_pos_traj(t_s, x_anchor, amp_x, amp_y, amp_z, rho, rho_dot, tones)
    quat_des, yaw, roll, tilt = _build_quat_traj(t_s, q_anchor_xyzw, amp_yaw, amp_roll, amp_rx, amp_ry, rho, tones)
    return x_des, dx_des, quat_des, yaw, roll, tilt
