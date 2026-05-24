#!/usr/bin/env python3
"""Generate a held-out multi-axis Cartesian excitation trajectory for step5c."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate multi-axis excitation target CSV/JSON for step5c.")
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
    parser.add_argument("--amp-x", type=float, default=0.04, help="X amplitude [m].")
    parser.add_argument("--amp-y", type=float, default=0.04, help="Y amplitude [m].")
    parser.add_argument("--amp-z", type=float, default=0.03, help="Z amplitude [m].")
    return parser.parse_args()


def _load_base_sidecar(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    for key in ("q_init", "x_anchor", "q_anchor_xyzw", "args"):
        if key not in payload:
            raise KeyError(f"{path} missing key: {key}")
    return payload


def _build_traj(
    t_s: np.ndarray,
    x_anchor: np.ndarray,
    amp_x: float,
    amp_y: float,
    amp_z: float,
) -> tuple[np.ndarray, np.ndarray]:
    two_pi = 2.0 * np.pi
    # 2-band trajectory for richer excitation.
    x = x_anchor[0] + amp_x * (
        np.sin(two_pi * 0.15 * t_s) + 0.4 * np.sin(two_pi * 0.7 * t_s)
    )
    y = x_anchor[1] + amp_y * (
        np.sin(two_pi * 0.20 * t_s + np.pi / 3.0) + 0.4 * np.sin(two_pi * 0.9 * t_s)
    )
    z = x_anchor[2] + amp_z * (
        np.sin(two_pi * 0.30 * t_s + np.pi / 4.0) + 0.4 * np.sin(two_pi * 1.1 * t_s)
    )

    dx = amp_x * (
        two_pi * 0.15 * np.cos(two_pi * 0.15 * t_s) + 0.4 * two_pi * 0.7 * np.cos(two_pi * 0.7 * t_s)
    )
    dy = amp_y * (
        two_pi * 0.20 * np.cos(two_pi * 0.20 * t_s + np.pi / 3.0)
        + 0.4 * two_pi * 0.9 * np.cos(two_pi * 0.9 * t_s)
    )
    dz = amp_z * (
        two_pi * 0.30 * np.cos(two_pi * 0.30 * t_s + np.pi / 4.0)
        + 0.4 * two_pi * 1.1 * np.cos(two_pi * 1.1 * t_s)
    )
    x_des = np.column_stack((x, y, z))
    dx_des = np.column_stack((dx, dy, dz))
    return x_des, dx_des


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
    x_des, dx_des = _build_traj(t_s, x_anchor, args.amp_x, args.amp_y, args.amp_z)
    quat_des = np.repeat(q_anchor_xyzw.reshape(1, 4), n, axis=0)

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    header = "t_s,x_des_x,x_des_y,x_des_z,dx_des_x,dx_des_y,dx_des_z,quat_des_x,quat_des_y,quat_des_z,quat_des_w"
    mat = np.column_stack((t_s, x_des, dx_des, quat_des))
    np.savetxt(out_csv, mat, delimiter=",", header=header, comments="", fmt="%.9f")

    args_ref = base.get("args", {})
    payload = {
        "schema_version": 1,
        "controller": "step5c_excitation_target",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "target_csv_path": str(out_csv),
        "base_sidecar_path": str(base_sidecar),
        "hz": int(args.hz),
        "duration_s": float(args.duration),
        "num_samples": int(n),
        "amplitude_m": {"x": float(args.amp_x), "y": float(args.amp_y), "z": float(args.amp_z)},
        "freq_set_hz": {
            "x": [0.15, 0.7],
            "y": [0.20, 0.9],
            "z": [0.30, 1.1],
        },
        "phase_rad": {"x": [0.0, 0.0], "y": [np.pi / 3.0, 0.0], "z": [np.pi / 4.0, 0.0]},
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

    print(f"[gen_excitation_traj] wrote {out_csv}")
    print(f"[gen_excitation_traj] wrote {out_sidecar}")
    print(
        "[gen_excitation_traj] peak |dx_des| [m/s]: "
        f"x={np.max(np.abs(dx_des[:,0])):.4f}, y={np.max(np.abs(dx_des[:,1])):.4f}, z={np.max(np.abs(dx_des[:,2])):.4f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
