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
    parser.add_argument(
        "--amp-ramp",
        type=float,
        default=2.0,
        help=(
            "Smooth half-cosine envelope length [s]. Position offset and "
            "velocity start at zero and reach the unenveloped trajectory at "
            "t = amp_ramp. Use 0 to disable (NOT recommended on the real robot)."
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


def _build_traj(
    t_s: np.ndarray,
    x_anchor: np.ndarray,
    amp_x: float,
    amp_y: float,
    amp_z: float,
    amp_ramp_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    two_pi = 2.0 * np.pi

    # 2-band base trajectory (relative to anchor) for richer excitation.
    bx = amp_x * (
        np.sin(two_pi * 0.15 * t_s) + 0.4 * np.sin(two_pi * 0.7 * t_s)
    )
    by = amp_y * (
        np.sin(two_pi * 0.20 * t_s + np.pi / 3.0) + 0.4 * np.sin(two_pi * 0.9 * t_s)
    )
    bz = amp_z * (
        np.sin(two_pi * 0.30 * t_s + np.pi / 4.0) + 0.4 * np.sin(two_pi * 1.1 * t_s)
    )

    dbx = amp_x * (
        two_pi * 0.15 * np.cos(two_pi * 0.15 * t_s)
        + 0.4 * two_pi * 0.7 * np.cos(two_pi * 0.7 * t_s)
    )
    dby = amp_y * (
        two_pi * 0.20 * np.cos(two_pi * 0.20 * t_s + np.pi / 3.0)
        + 0.4 * two_pi * 0.9 * np.cos(two_pi * 0.9 * t_s)
    )
    dbz = amp_z * (
        two_pi * 0.30 * np.cos(two_pi * 0.30 * t_s + np.pi / 4.0)
        + 0.4 * two_pi * 1.1 * np.cos(two_pi * 1.1 * t_s)
    )

    # Smooth amplitude envelope so x_des(0) = x_anchor (kills the initial
    # 3.5 cm |e_pos|_inf coming from the y/z phase offsets) and dx_des(0) = 0
    # (no velocity feedforward kick at t=0). Also gives a smooth ramp-out
    # at t = amp_ramp so there is no acceleration spike when full amplitude
    # is reached.
    rho, rho_dot = _half_cosine_envelope(t_s, amp_ramp_s)

    x = x_anchor[0] + rho * bx
    y = x_anchor[1] + rho * by
    z = x_anchor[2] + rho * bz

    dx = rho_dot * bx + rho * dbx
    dy = rho_dot * by + rho * dby
    dz = rho_dot * bz + rho * dbz

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
    x_des, dx_des = _build_traj(
        t_s,
        x_anchor,
        args.amp_x,
        args.amp_y,
        args.amp_z,
        float(args.amp_ramp),
    )
    quat_des = np.repeat(q_anchor_xyzw.reshape(1, 4), n, axis=0)

    initial_offset = np.linalg.norm(x_des[0] - x_anchor)
    if initial_offset > 1e-6:
        raise ValueError(
            f"sanity check failed: |x_des(0) - x_anchor| = {initial_offset:.6e} m"
        )

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
        "amp_ramp_s": float(args.amp_ramp),
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
