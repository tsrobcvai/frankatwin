#!/usr/bin/env python3
"""Generate the SysID v3 multi-band excitation trajectory (Franka).

Writes a 1 kHz target CSV + sidecar JSON. The math lives in
``frankatwin.excitation.multiband``; this file is the command line around it.

Run:
  python scripts/gen_excitation_traj.py --base-sidecar ref.json [--out-csv …] [--out-sidecar …]
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from frankatwin.excitation.multiband import (  # noqa: E402
    ORI_FREQS,
    POS_FREQS,
    _load_base_sidecar,
    build_multiband_trajectory,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the SysID v3 multi-band excitation target CSV/JSON.")
    parser.add_argument(
        "--base-sidecar",
        type=str,
        required=True,
        help="Reference sidecar JSON (from read_current_pose or a previous run) "
             "providing q_init / x_anchor / q_anchor and gain hints.",
    )
    parser.add_argument(
        "--out-csv",
        type=str,
        default="data/multiband_target.csv",
        help="Output target CSV path.",
    )
    parser.add_argument(
        "--out-sidecar",
        type=str,
        default="data/multiband_target.json",
        help="Output sidecar JSON path.",
    )
    parser.add_argument("--duration", type=float, default=12.0, help="Trajectory duration in seconds.")
    parser.add_argument("--hz", type=int, default=1000, help="Sampling frequency.")
    parser.add_argument("--amp-x", type=float, default=0.10, help="X amplitude [m] (low-band).")
    parser.add_argument("--amp-y", type=float, default=0.10, help="Y amplitude [m] (low-band).")
    parser.add_argument("--amp-z", type=float, default=0.08, help="Z amplitude [m] (low-band).")
    parser.add_argument(
        "--amp-yaw",
        type=float,
        default=0.25,
        help="Yaw (rotation about world z, drives j1) amplitude [rad]. 0 = orientation held (position-only v1/v2 variant).",
    )
    parser.add_argument(
        "--amp-roll",
        type=float,
        default=0.20,
        help="Roll (rotation about EE z, drives j5/j7) amplitude [rad]. 0 = orientation held.",
    )
    parser.add_argument(
        "--high-band-ratio",
        type=float,
        default=0.20,
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
    x_des, dx_des, quat_des, yaw_traj, roll_traj = build_multiband_trajectory(
        t_s,
        x_anchor,
        q_anchor_xyzw,
        amp_x=args.amp_x,
        amp_y=args.amp_y,
        amp_z=args.amp_z,
        amp_yaw=float(args.amp_yaw),
        amp_roll=float(args.amp_roll),
        high_band_ratio=float(args.high_band_ratio),
        amp_ramp_s=float(args.amp_ramp),
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

    # Peak rate diagnostics against robot.yaml's default per-tick position
    # clamp (error_delta_pos = 0.05 m) and the 0.30 m/s Cartesian speed
    # convention.
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
        "controller": "multiband_excitation_target" if (args.amp_yaw > 0 or args.amp_roll > 0) else "multiband_pos_only_target",
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

    # Convention limits: peak Cartesian speed 0.30 m/s, peak angular rate
    # 0.50 rad/s.  Nothing enforces them at runtime (osc_shm only clamps the
    # position tracking error), but staying inside the envelope is the
    # conservative thing to do.
    CART_DX_PEAK_LIMIT_MPS = 0.30
    ORI_DOT_PEAK_LIMIT_RPS = 0.50
    peak_ori_combined = float(args.amp_yaw + args.amp_roll) * (1.0 + float(args.high_band_ratio))

    print(f"[gen_excitation_traj] wrote {out_csv}")
    print(f"[gen_excitation_traj] wrote {out_sidecar}")
    print(
        f"[gen_excitation_traj] peak |dx_des| [m/s]: x={peak_dx:.4f}, y={peak_dy:.4f}, z={peak_dz:.4f}  "
        f"(|dx|_max={peak_speed_cart:.4f} m/s, convention {CART_DX_PEAK_LIMIT_MPS:.2f})"
    )
    print(
        "[gen_excitation_traj] peak rotation rates [rad/s]: "
        f"dyaw={peak_dyaw:.4f}, droll={peak_droll:.4f} "
        f"(convention {ORI_DOT_PEAK_LIMIT_RPS:.2f}). "
        f"yaw amp={args.amp_yaw:.3f} rad, roll amp={args.amp_roll:.3f} rad. "
        f"q_des max ang-offset from anchor ~ {peak_ori_combined:.3f} rad"
    )
    if peak_speed_cart > CART_DX_PEAK_LIMIT_MPS:
        print(
            f"[gen_excitation_traj] WARNING: peak Cartesian speed {peak_speed_cart:.3f} m/s "
            f"> convention {CART_DX_PEAK_LIMIT_MPS:.2f} m/s.  Reduce --amp-* or --high-band-ratio."
        )
    if max(peak_dyaw, peak_droll) > ORI_DOT_PEAK_LIMIT_RPS:
        print(
            f"[gen_excitation_traj] WARNING: peak angular rate {max(peak_dyaw, peak_droll):.3f} rad/s "
            f"> convention {ORI_DOT_PEAK_LIMIT_RPS:.2f} rad/s.  "
            "Reduce --amp-yaw/--amp-roll or --high-band-ratio."
        )
    return 0



if __name__ == "__main__":
    raise SystemExit(main())
