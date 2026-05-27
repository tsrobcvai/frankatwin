#!/usr/bin/env python3
"""Generate the SysID v4 chirp excitation trajectory (Franka).

v4 supersedes v3 (step5d, see gen_excitation_traj.py) by switching from a
two-band stationary sinusoid to a linear frequency sweep ("chirp"). The
design follows UR5e's collect_sysid_data.py (linear chirp, 6-DOF, evenly
phase-staggered) but with bounds tuned for the Franka task-impedance loop.

Key differences vs v3 (step5d):

  * Frequency profile: linear chirp f0->f1 instead of two fixed bands per axis.
    This sweeps every frequency between f0 and f1 once, giving CMA-ES a much
    denser spectral footprint to fit armature / viscous / delay against.

  * All 6 Cartesian DOFs active simultaneously (x, y, z, rx, ry, rz) with
    phase offsets at k * pi/3.  v3 left rotation off by default; v4 always
    excites rotation so J5/J6/J7 friction is identifiable.

  * Z amplitude > XY amplitude (v3 had Z smallest).  Vertical motion couples
    into J2/J3 gravity terms, so giving Z more energy improves identification
    of shoulder/elbow dynamics.

  * Asymmetric linear ramp (1.5s up / 2.5s down) matching UR5e's convention.

The sidecar JSON written by this script is compatible with apply_sysid_params
(``--invoke-replay``) and replay_real_step5b_sim.py for downstream sim/real
comparison.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# Defaults follow the UR5e chirp template (collect_sysid_data.py) but scaled
# for the Franka task-impedance envelope.  The base step5b sidecar conventions
# (peak Cartesian speed <= 0.30 m/s, peak angular rate <= 0.50 rad/s) still
# apply -- see _print_peak_rates for the runtime warnings.
# ---------------------------------------------------------------------------

# Defaults follow UR5e's collect_sysid_data.py *shape* exactly (amplitudes,
# Z=1.5xXY ratio, asymmetric 2s/3s ramp, pi/3 phase offsets, 8 s duration)
# but with f1 halved from 3.0 -> 1.5 Hz to keep peak |dx| ~ 1 m/s instead of
# ~2 m/s.  Halving f1 keeps the per-axis spectral *shape* (linear chirp from
# 0.1 Hz to f1) but compresses the band to where Franka's task-impedance
# loop can actually track and the 50 Hz cart_impedance.py logger is well
# above Nyquist.
#
# IMPORTANT: With UR5e-amp rotations (0.50 + 0.25 + 0.50 rad), the reference
# itself reaches max|rot_offset| ~= 0.61 rad.  This is *above* the 0.40 rad
# ori_track_abort_rad used by the C++ step5d_excite collector -- if you
# collect with that pipeline you MUST raise the abort to >= 0.80.  The
# Python cart_impedance.py path does not enforce that abort (only prints
# warnings), so it is unaffected.
CHIRP_F0_DEFAULT = 0.1
CHIRP_F1_DEFAULT = 1.5
# Per-axis phase offsets (6 axes, 6 evenly spaced offsets k * pi/3).
PHASE_OFFSETS = np.array([0.0, np.pi / 3.0, 2.0 * np.pi / 3.0,
                          np.pi, 4.0 * np.pi / 3.0, 5.0 * np.pi / 3.0])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate v4 chirp excitation target (CSV + sidecar JSON).")
    parser.add_argument(
        "--base-sidecar",
        type=str,
        default="/home/tao/Projects/panda_control/data/step5b_20260524_120834.json",
        help="Reference real sidecar used for q_init / x_anchor / q_anchor and gain hints.",
    )
    parser.add_argument(
        "--out-csv",
        type=str,
        default="/home/tao/Projects/panda_control/tmp/v4_chirp_target.csv",
        help="Output target CSV path.",
    )
    parser.add_argument(
        "--out-sidecar",
        type=str,
        default="/home/tao/Projects/panda_control/tmp/v4_chirp_target.json",
        help="Output sidecar JSON path.",
    )
    parser.add_argument("--duration", type=float, default=8.0, help="Trajectory duration in seconds (UR5e default).")
    parser.add_argument("--hz", type=int, default=1000, help="CSV sample rate (1000 matches the C++ collector schema).")
    # Frequency sweep
    parser.add_argument("--f0", type=float, default=CHIRP_F0_DEFAULT, help="Chirp start frequency [Hz].")
    parser.add_argument("--f1", type=float, default=CHIRP_F1_DEFAULT, help="Chirp end frequency [Hz] (UR5e=3.0, we halve to 1.5 for Franka).")
    # Cartesian amplitudes (m) -- match UR5e collect_sysid_data exactly.
    # UR5e per-axis = [pos_amp, pos_amp, pos_amp * 1.5] with pos_amp=0.10.
    parser.add_argument("--amp-x", type=float, default=0.10, help="X amplitude [m] (UR5e default).")
    parser.add_argument("--amp-y", type=float, default=0.10, help="Y amplitude [m] (UR5e default).")
    parser.add_argument("--amp-z", type=float, default=0.15, help="Z amplitude [m] (UR5e default, 1.5x XY).")
    # Rotation amplitudes (rad) -- match UR5e exactly.
    # UR5e per-axis = [rot_amp * 2, rot_amp, rot_amp * 2] with rot_amp=0.25.
    # NOTE: max|rot_offset| can reach ~0.61 rad, above the 0.40 rad C++ collector
    # abort.  cart_impedance.py path is unaffected; bump that abort if you
    # switch to the C++ collector.
    parser.add_argument("--amp-rx", type=float, default=0.50, help="World-x rotation amplitude [rad] (UR5e default).")
    parser.add_argument("--amp-ry", type=float, default=0.25, help="World-y rotation amplitude [rad] (UR5e default; drives J6).")
    parser.add_argument("--amp-rz", type=float, default=0.50, help="World-z rotation amplitude [rad] (UR5e default; drives J1).")
    # Ramp envelope (linear, asymmetric) -- match UR5e (2s up, 3s down).
    parser.add_argument("--ramp-up", type=float, default=2.0, help="Linear ramp-up length [s] (UR5e default).")
    parser.add_argument("--ramp-down", type=float, default=3.0, help="Linear ramp-down length [s] (UR5e default).")
    return parser.parse_args()


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
        x_des:           (T, 3) Cartesian target positions.
        dx_des:          (T, 3) Cartesian target velocities (analytic).
        quat_des_xyzw:   (T, 4) target orientation (xyzw), anchor pre-rotated
                         by the world-frame axis-angle offset.
        rot_offsets:     (T, 3) axis-angle offset vector applied to the anchor
                         (returned for diagnostics + sidecar metadata).
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


# Safety conventions (from step5b/step5d collector pre-flights).
CART_DX_PEAK_LIMIT_MPS = 0.30
ORI_DOT_PEAK_LIMIT_RPS = 0.50
ORI_TRACK_ABORT_RAD = 0.30


def main() -> int:
    args = parse_args()
    base_sidecar = Path(args.base_sidecar).expanduser().resolve()
    out_csv = Path(args.out_csv).expanduser().resolve()
    out_sidecar = Path(args.out_sidecar).expanduser().resolve()

    base = _load_base_sidecar(base_sidecar)
    x_anchor = np.asarray(base["x_anchor"], dtype=np.float64)
    q_anchor_xyzw = np.asarray(base["q_anchor_xyzw"], dtype=np.float64)
    q_norm = float(np.linalg.norm(q_anchor_xyzw))
    if q_norm < 1e-12:
        raise ValueError("q_anchor_xyzw norm is too small.")
    q_anchor_xyzw = q_anchor_xyzw / q_norm

    dt = 1.0 / float(args.hz)
    n = int(round(float(args.duration) * float(args.hz))) + 1
    t_s = np.arange(n, dtype=np.float64) * dt

    x_des, dx_des, quat_des, rot_offsets = build_chirp_trajectory(
        t_s,
        x_anchor,
        q_anchor_xyzw,
        f0=float(args.f0),
        f1=float(args.f1),
        amp_x=float(args.amp_x),
        amp_y=float(args.amp_y),
        amp_z=float(args.amp_z),
        amp_rx=float(args.amp_rx),
        amp_ry=float(args.amp_ry),
        amp_rz=float(args.amp_rz),
        ramp_up_s=float(args.ramp_up),
        ramp_down_s=float(args.ramp_down),
    )

    # Sanity: anchor recovered at t=0 (rho(0)=0 -> offsets all zero).
    initial_offset = float(np.linalg.norm(x_des[0] - x_anchor))
    if initial_offset > 1e-6:
        raise ValueError(f"sanity check failed: |x_des(0) - x_anchor| = {initial_offset:.6e} m")
    dot0 = float(abs(np.dot(quat_des[0], q_anchor_xyzw)))
    if dot0 < 1.0 - 1e-6:
        raise ValueError(f"sanity check failed: |q_des(0) . q_anchor| = {dot0:.6e} (expected ~1)")

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "t_s,x_des_x,x_des_y,x_des_z,dx_des_x,dx_des_y,dx_des_z,"
        "quat_des_x,quat_des_y,quat_des_z,quat_des_w,"
        "rot_off_x,rot_off_y,rot_off_z"
    )
    mat = np.column_stack((t_s, x_des, dx_des, quat_des, rot_offsets))
    np.savetxt(out_csv, mat, delimiter=",", header=header, comments="", fmt="%.9f")

    peak = _peak_rates(t_s, dx_des, rot_offsets)
    # Max angular offset from anchor (axis-angle magnitude bound).
    peak_ori_offset = float(np.max(np.linalg.norm(rot_offsets, axis=1)))

    args_ref = base.get("args", {})
    payload = {
        "schema_version": 2,
        "controller": "v4_chirp_excitation_target",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "target_csv_path": str(out_csv),
        "base_sidecar_path": str(base_sidecar),
        "hz": int(args.hz),
        "duration_s": float(args.duration),
        "num_samples": int(n),
        "chirp": {
            "f0_hz": float(args.f0),
            "f1_hz": float(args.f1),
            "phase_offsets_rad": PHASE_OFFSETS.tolist(),
            "axes_order": ["x", "y", "z", "rx", "ry", "rz"],
        },
        "amplitude_m": {"x": float(args.amp_x), "y": float(args.amp_y), "z": float(args.amp_z)},
        "amplitude_rad": {"rx": float(args.amp_rx), "ry": float(args.amp_ry), "rz": float(args.amp_rz)},
        "ramp": {"up_s": float(args.ramp_up), "down_s": float(args.ramp_down)},
        "peak_rates": peak,
        "peak_ori_offset_rad": peak_ori_offset,
        "q_init": base["q_init"],
        "x_anchor": x_anchor.tolist(),
        "q_anchor_xyzw": q_anchor_xyzw.tolist(),
        # Recommended v4 controller gains.  Note these are LOWER than UR5e's
        # (kp_pos=1000, kp_rot=50) because the Franka task-impedance loop
        # behaves differently and 500/30 is the operating point you chose for
        # sim2real parity.  The actual gains at collection time must be
        # passed via --kp-pos / --kp-ori CLI to cart_impedance.py and will be
        # re-recorded in the live-run sidecar.
        "recommended_args": {
            "kp_pos": 500.0,
            "kp_ori": 30.0,
            "kd_pos": 2.0 * np.sqrt(500.0),
            "kd_ori": 2.0 * np.sqrt(30.0),
            "no_coriolis": bool(args_ref.get("no_coriolis", False)),
        },
    }
    out_sidecar.parent.mkdir(parents=True, exist_ok=True)
    out_sidecar.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(f"[gen_chirp_traj] wrote {out_csv}")
    print(f"[gen_chirp_traj] wrote {out_sidecar}")
    print(
        f"[gen_chirp_traj] peak |dx_des| [m/s]: x={peak['dx_m_s'][0]:.4f}, "
        f"y={peak['dx_m_s'][1]:.4f}, z={peak['dx_m_s'][2]:.4f}  "
        f"(|dx|_max={peak['cart_speed_m_s']:.4f}, convention {CART_DX_PEAK_LIMIT_MPS:.2f})"
    )
    print(
        f"[gen_chirp_traj] peak rotation rates [rad/s]: "
        f"drx={peak['drot_rad_s'][0]:.4f}, dry={peak['drot_rad_s'][1]:.4f}, "
        f"drz={peak['drot_rad_s'][2]:.4f}  (convention {ORI_DOT_PEAK_LIMIT_RPS:.2f}). "
        f"max |rot_offset| ~ {peak_ori_offset:.3f} rad "
        f"(runtime abort {ORI_TRACK_ABORT_RAD:.2f})"
    )

    if peak["cart_speed_m_s"] > CART_DX_PEAK_LIMIT_MPS:
        print(
            f"[gen_chirp_traj] WARNING: peak Cartesian speed "
            f"{peak['cart_speed_m_s']:.3f} > {CART_DX_PEAK_LIMIT_MPS:.2f} m/s. "
            "Reduce --amp-x/y/z or --f1."
        )
    peak_ang = peak["ang_rate_rad_s"]
    if peak_ang > ORI_DOT_PEAK_LIMIT_RPS:
        print(
            f"[gen_chirp_traj] WARNING: peak angular rate {peak_ang:.3f} > "
            f"{ORI_DOT_PEAK_LIMIT_RPS:.2f} rad/s. Reduce --amp-rx/ry/rz or --f1."
        )
    if peak_ori_offset > 0.8 * ORI_TRACK_ABORT_RAD:
        print(
            f"[gen_chirp_traj] WARNING: max |rot_offset| = {peak_ori_offset:.3f} rad "
            f"is > 80% of the {ORI_TRACK_ABORT_RAD:.2f} rad runtime abort. "
            "If the controller lags, q_err can trip the abort."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
