#!/usr/bin/env python3
"""Generate the SysID v4 chirp excitation trajectory (Franka).

Writes a 1 kHz target CSV + sidecar JSON. The math lives in
``frankatwin.excitation.chirp``; this file is the command line around it.

Run:
  python scripts/gen_chirp_traj.py --base-sidecar ref.json [--out-csv …] [--out-sidecar …]
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from frankatwin.excitation.chirp import (  # noqa: E402
    CART_DX_PEAK_LIMIT_MPS,
    CHIRP_F0_DEFAULT,
    CHIRP_F1_DEFAULT,
    ORI_DOT_PEAK_LIMIT_RPS,
    PHASE_OFFSETS,
    _load_base_sidecar,
    _peak_rates,
    build_chirp_trajectory,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate v4 chirp excitation target (CSV + sidecar JSON).")
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
        default="data/chirp_target.csv",
        help="Output target CSV path.",
    )
    parser.add_argument(
        "--out-sidecar",
        type=str,
        default="data/chirp_target.json",
        help="Output sidecar JSON path.",
    )
    parser.add_argument("--duration", type=float, default=8.0, help="Trajectory duration in seconds (UR5e default).")
    parser.add_argument("--hz", type=int, default=1000, help="CSV sample rate (1000 matches the C++ collector schema).")
    # Frequency sweep
    parser.add_argument("--f0", type=float, default=CHIRP_F0_DEFAULT, help="Chirp start frequency [Hz].")
    parser.add_argument("--f1", type=float, default=CHIRP_F1_DEFAULT, help="Chirp end frequency [Hz] (UR5e=3.0; Franka production default=0.7).")
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
        f"max |rot_offset| ~ {peak_ori_offset:.3f} rad"
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
    return 0



if __name__ == "__main__":
    raise SystemExit(main())
