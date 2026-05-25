#!/usr/bin/env python3
"""Generate a held-out multi-axis Cartesian + orientation excitation trajectory.

History:

  * step5c (sysid v1/v2): position-only multi-band sweep, quaternion held at
    ``q_anchor``.  This left j1 (base yaw) and j5 (wrist roll) under-excited
    because the task wrench had no torque component.
  * step5d (sysid v3, this file): adds two extra multi-band rotation sweeps,
    one about base-z (drives j1 directly) and one about EE-z (drives j5/j7).
    Position amplitudes are also bumped (default 10 cm / 8 cm).  Both
    rotations are off by default (``--amp-yaw 0 --amp-roll 0``), so existing
    step5c command lines are still bit-identical.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate multi-axis excitation target CSV/JSON (step5c/step5d).")
    parser.add_argument(
        "--base-sidecar",
        type=str,
        default="/home/tao/Projects/panda_control/data/step5b_20260524_120834.json",
        help="Reference real sidecar used for q_init/x_anchor/q_anchor and gains.",
    )
    parser.add_argument(
        "--out-csv",
        type=str,
        default="/home/tao/Projects/panda_control/tmp/step5c_excitation_target.csv",
        help="Output target CSV path.",
    )
    parser.add_argument(
        "--out-sidecar",
        type=str,
        default="/home/tao/Projects/panda_control/tmp/step5c_excitation_target.json",
        help="Output sidecar JSON path.",
    )
    parser.add_argument("--duration", type=float, default=12.0, help="Trajectory duration in seconds.")
    parser.add_argument("--hz", type=int, default=1000, help="Sampling frequency.")
    parser.add_argument("--amp-x", type=float, default=0.04, help="X amplitude [m] (low-band).")
    parser.add_argument("--amp-y", type=float, default=0.04, help="Y amplitude [m] (low-band).")
    parser.add_argument("--amp-z", type=float, default=0.03, help="Z amplitude [m] (low-band).")
    parser.add_argument(
        "--amp-yaw",
        type=float,
        default=0.0,
        help="Yaw (rotation about world z, drives j1) amplitude [rad]. 0 = orientation held (step5c behavior).",
    )
    parser.add_argument(
        "--amp-roll",
        type=float,
        default=0.0,
        help="Roll (rotation about EE z, drives j5/j7) amplitude [rad]. 0 = orientation held.",
    )
    parser.add_argument(
        "--high-band-ratio",
        type=float,
        default=0.4,
        help="High-band amplitude as a fraction of low-band amplitude. Lower this if peak speeds approach safety limits.",
    )
    parser.add_argument(
        "--amp-ramp",
        type=float,
        default=2.0,
        help=(
            "Smooth half-cosine envelope length [s]. Position/orientation "
            "offsets and rates start at zero and reach the unenveloped "
            "trajectory at t = amp_ramp. Use 0 to disable (NOT recommended on the real robot)."
        ),
    )
    return parser.parse_args()


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
    Both default to amp=0, in which case q_des(t) = q_anchor (step5c parity).
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


def _build_traj(
    t_s: np.ndarray,
    x_anchor: np.ndarray,
    q_anchor_xyzw: np.ndarray,
    amp_x: float,
    amp_y: float,
    amp_z: float,
    amp_yaw: float,
    amp_roll: float,
    high_band_ratio: float,
    amp_ramp_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # Smooth amplitude envelope so x_des(0) = x_anchor, dx_des(0) = 0, and the
    # rotation also starts at q_anchor with zero angular velocity feedforward.
    rho, rho_dot = _half_cosine_envelope(t_s, amp_ramp_s)
    x_des, dx_des = _build_pos_traj(t_s, x_anchor, amp_x, amp_y, amp_z, rho, rho_dot, high_band_ratio)
    quat_des, yaw, roll = _build_quat_traj(t_s, q_anchor_xyzw, amp_yaw, amp_roll, rho, high_band_ratio)
    return x_des, dx_des, quat_des, yaw, roll


def main() -> int:
    args = parse_args()
    base_sidecar = Path(args.base_sidecar).expanduser().resolve()
    out_csv = Path(args.out_csv).expanduser().resolve()
    out_sidecar = Path(args.out_sidecar).expanduser().resolve()

    base = _load_base_sidecar(base_sidecar)
    x_anchor = np.asarray(base["x_anchor"], dtype=np.float64)
    q_anchor_xyzw = np.asarray(base["q_anchor_xyzw"], dtype=np.float64)
    q_norm = np.linalg.norm(q_anchor_xyzw)
    if q_norm < 1e-12:
        raise ValueError("q_anchor_xyzw norm is too small.")
    q_anchor_xyzw = q_anchor_xyzw / q_norm

    dt = 1.0 / float(args.hz)
    n = int(round(float(args.duration) * float(args.hz))) + 1
    t_s = np.arange(n, dtype=np.float64) * dt
    x_des, dx_des, quat_des, yaw_traj, roll_traj = _build_traj(
        t_s,
        x_anchor,
        q_anchor_xyzw,
        args.amp_x,
        args.amp_y,
        args.amp_z,
        float(args.amp_yaw),
        float(args.amp_roll),
        float(args.high_band_ratio),
        float(args.amp_ramp),
    )

    initial_offset = np.linalg.norm(x_des[0] - x_anchor)
    if initial_offset > 1e-6:
        raise ValueError(
            f"sanity check failed: |x_des(0) - x_anchor| = {initial_offset:.6e} m"
        )
    # quat_des(0) must equal q_anchor (within sign): yaw(0) = roll(0) = 0
    # by the ramp envelope, so q_yaw_base = q_roll_local = identity.
    q0 = quat_des[0]
    dot = float(abs(np.dot(q0, q_anchor_xyzw)))
    if dot < 1.0 - 1e-6:
        raise ValueError(
            f"sanity check failed: |q_des(0) · q_anchor| = {dot:.6e} (expected ~1.0)"
        )

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    header = "t_s,x_des_x,x_des_y,x_des_z,dx_des_x,dx_des_y,dx_des_z,quat_des_x,quat_des_y,quat_des_z,quat_des_w"
    mat = np.column_stack((t_s, x_des, dx_des, quat_des))
    np.savetxt(out_csv, mat, delimiter=",", header=header, comments="", fmt="%.9f")

    args_ref = base.get("args", {})

    # Peak rate diagnostics (for safety thresholds in step5c_excite.cpp:
    # CART_TRACK_ABORT_M = 0.05 m, ORI_TRACK_ABORT_RAD = 0.30 rad,
    # plus the hard-coded Cartesian speed limit ~0.3 m/s in step5b_cart_pose).
    peak_dx = float(np.max(np.abs(dx_des[:, 0])))
    peak_dy = float(np.max(np.abs(dx_des[:, 1])))
    peak_dz = float(np.max(np.abs(dx_des[:, 2])))
    peak_dyaw = float(np.max(np.abs(np.gradient(yaw_traj, dt))))
    peak_droll = float(np.max(np.abs(np.gradient(roll_traj, dt))))
    peak_speed_cart = float(np.max(np.linalg.norm(dx_des, axis=1)))

    pos_freq_set = {axis: [POS_FREQS[axis][0], POS_FREQS[axis][1]] for axis in ("x", "y", "z")}
    pos_phase_set = {axis: [POS_FREQS[axis][2], 0.0] for axis in ("x", "y", "z")}
    ori_freq_set = {axis: [ORI_FREQS[axis][0], ORI_FREQS[axis][1]] for axis in ("yaw", "roll")}
    ori_phase_set = {axis: [ORI_FREQS[axis][2], 0.0] for axis in ("yaw", "roll")}

    payload = {
        "schema_version": 2,
        "controller": "step5d_excitation_target" if (args.amp_yaw > 0 or args.amp_roll > 0) else "step5c_excitation_target",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "target_csv_path": str(out_csv),
        "base_sidecar_path": str(base_sidecar),
        "hz": int(args.hz),
        "duration_s": float(args.duration),
        "num_samples": int(n),
        "amp_ramp_s": float(args.amp_ramp),
        "high_band_ratio": float(args.high_band_ratio),
        "amplitude_m": {"x": float(args.amp_x), "y": float(args.amp_y), "z": float(args.amp_z)},
        "amplitude_rad": {"yaw": float(args.amp_yaw), "roll": float(args.amp_roll)},
        "freq_set_hz": pos_freq_set,
        "phase_rad": pos_phase_set,
        "ori_freq_set_hz": ori_freq_set,
        "ori_phase_rad": ori_phase_set,
        "peak_rates": {
            "cart_speed_m_s": peak_speed_cart,
            "dx_m_s": [peak_dx, peak_dy, peak_dz],
            "dyaw_rad_s": peak_dyaw,
            "droll_rad_s": peak_droll,
        },
        "q_init": base["q_init"],
        "x_anchor": x_anchor.tolist(),
        "q_anchor_xyzw": q_anchor_xyzw.tolist(),
        "args": {
            "kp_pos": float(args_ref.get("kp_pos", 200.0)),
            "kd_pos": float(args_ref.get("kd_pos", 2.0 * np.sqrt(float(args_ref.get("kp_pos", 200.0))))),
            "kp_ori": float(args_ref.get("kp_ori", 20.0)),
            "kd_ori": float(args_ref.get("kd_ori", 2.0 * np.sqrt(float(args_ref.get("kp_ori", 20.0))))),
            "ramp": float(args_ref.get("ramp", 1.5)),
            "amp_ramp": float(args_ref.get("amp_ramp", args_ref.get("ramp", 1.5))),
            "no_coriolis": bool(args_ref.get("no_coriolis", False)),
        },
    }
    out_sidecar.parent.mkdir(parents=True, exist_ok=True)
    out_sidecar.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    # Convention limits.  step5b_cart_pose.cpp pre-flights both peak speeds at
    # 0.30 m/s / 0.50 rad/s.  step5c_excite.cpp does NOT runtime-enforce them
    # (only the tracking-error aborts at 5 cm / 0.30 rad), but staying inside
    # the step5b envelope is the conservative thing to do.
    CART_DX_PEAK_LIMIT_MPS = 0.30
    ORI_DOT_PEAK_LIMIT_RPS = 0.50
    ORI_TRACK_ABORT_RAD = 0.30
    peak_ori_combined = float(args.amp_yaw + args.amp_roll) * (1.0 + float(args.high_band_ratio))

    print(f"[gen_excitation_traj] wrote {out_csv}")
    print(f"[gen_excitation_traj] wrote {out_sidecar}")
    print(
        f"[gen_excitation_traj] peak |dx_des| [m/s]: x={peak_dx:.4f}, y={peak_dy:.4f}, z={peak_dz:.4f}  "
        f"(|dx|_max={peak_speed_cart:.4f} m/s, step5b convention {CART_DX_PEAK_LIMIT_MPS:.2f})"
    )
    print(
        "[gen_excitation_traj] peak rotation rates [rad/s]: "
        f"dyaw={peak_dyaw:.4f}, droll={peak_droll:.4f} "
        f"(step5b convention {ORI_DOT_PEAK_LIMIT_RPS:.2f}). "
        f"yaw amp={args.amp_yaw:.3f} rad, roll amp={args.amp_roll:.3f} rad. "
        f"q_des max ang-offset from anchor ~ {peak_ori_combined:.3f} rad "
        f"(runtime abort fires when |q_err| > {ORI_TRACK_ABORT_RAD:.2f} rad)"
    )
    if peak_speed_cart > CART_DX_PEAK_LIMIT_MPS:
        print(
            f"[gen_excitation_traj] WARNING: peak Cartesian speed {peak_speed_cart:.3f} m/s "
            f"> step5b convention {CART_DX_PEAK_LIMIT_MPS:.2f} m/s.  Reduce --amp-* or --high-band-ratio."
        )
    if max(peak_dyaw, peak_droll) > ORI_DOT_PEAK_LIMIT_RPS:
        print(
            f"[gen_excitation_traj] WARNING: peak angular rate {max(peak_dyaw, peak_droll):.3f} rad/s "
            f"> step5b convention {ORI_DOT_PEAK_LIMIT_RPS:.2f} rad/s.  "
            "Reduce --amp-yaw/--amp-roll or --high-band-ratio."
        )
    if peak_ori_combined > 0.8 * ORI_TRACK_ABORT_RAD:
        print(
            f"[gen_excitation_traj] WARNING: amp_yaw + amp_roll = {peak_ori_combined:.3f} rad "
            f"is > 80% of the {ORI_TRACK_ABORT_RAD:.2f} rad runtime abort.  "
            "If the controller lags much (e.g. >25%), q_err can trip the abort."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
