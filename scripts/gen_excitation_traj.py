#!/usr/bin/env python3
"""Generate the multi-band excitation trajectory (Franka).

Writes a 1 kHz target CSV + sidecar JSON. The math lives in
``frankatwin.excitation.multiband``; this file is the command line around it.

``--profile`` picks the design (``MULTIBAND_PROFILES``) and the defaults of the
amplitude / envelope flags. One ships: ``heldout``, the 6-DOF held-out
validation run (tilt about world x / y plus yaw, three bands up to 2 Hz at
amplitudes ~ 0.35/f, fades out to the anchor).

Run:
  python scripts/gen_excitation_traj.py --base-sidecar ref.json [--profile heldout] [--out-csv …] [--out-sidecar …]
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from frankatwin.excitation.multiband import (  # noqa: E402
    MULTIBAND_PROFILES,
    _load_base_sidecar,
    build_multiband_trajectory,
    multiband_tones,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the multi-band excitation target CSV/JSON.")
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
        default=None,
        help="Output target CSV path (default: data/multiband_<profile>_target.csv).",
    )
    parser.add_argument(
        "--out-sidecar",
        type=str,
        default=None,
        help="Output sidecar JSON path (default: the CSV path with .json).",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(MULTIBAND_PROFILES),
        default="heldout",
        help="heldout = 6-DOF, three bands, fades out. "
             "Sets the defaults of the amplitude and envelope flags below.",
    )
    parser.add_argument("--duration", type=float, default=12.0, help="Trajectory duration in seconds.")
    parser.add_argument("--hz", type=int, default=1000, help="Sampling frequency.")
    parser.add_argument("--amp-x", type=float, default=None, help="X amplitude [m] of the low-band tone (default: 0.06).")
    parser.add_argument("--amp-y", type=float, default=None, help="Y amplitude [m] of the low-band tone (default: 0.06).")
    parser.add_argument("--amp-z", type=float, default=None, help="Z amplitude [m] of the low-band tone (default: 0.06).")
    parser.add_argument(
        "--amp-yaw",
        type=float,
        default=None,
        help="Yaw (rotation about world z) amplitude [rad] (default: 0.25). With roll, rx and ry at 0 too, "
             "0 = orientation held (position-only variant).",
    )
    parser.add_argument(
        "--amp-roll",
        type=float,
        default=None,
        help="Roll (rotation about EE z) amplitude [rad]; with the tool pointing down, the same rotation "
             "as yaw, reversed (default: 0).",
    )
    parser.add_argument("--amp-rx", type=float, default=None, help="Tilt about world x, amplitude [rad] (default: 0.20).")
    parser.add_argument("--amp-ry", type=float, default=None, help="Tilt about world y, amplitude [rad] (default: 0.15).")
    parser.add_argument(
        "--amp-ramp",
        type=float,
        default=None,
        help=(
            "Smooth half-cosine fade-in length [s] (default: 2.0). Position/orientation "
            "offsets and rates start at zero and reach the unenveloped "
            "trajectory at t = amp_ramp. Use 0 to disable (NOT recommended on the real robot)."
        ),
    )
    parser.add_argument(
        "--amp-ramp-down",
        type=float,
        default=None,
        help="Half-cosine fade-out length [s]; 0 = end at full amplitude (default: 2.0).",
    )
    args = parser.parse_args()
    prof = MULTIBAND_PROFILES[args.profile]
    for flag, key in (("amp_x", "amp_x"), ("amp_y", "amp_y"), ("amp_z", "amp_z"),
                      ("amp_yaw", "amp_yaw"), ("amp_roll", "amp_roll"),
                      ("amp_rx", "amp_rx"), ("amp_ry", "amp_ry"),
                      ("amp_ramp", "amp_ramp_s"), ("amp_ramp_down", "ramp_down_s")):
        if getattr(args, flag) is None:
            setattr(args, flag, prof[key])
    if args.out_csv is None:
        args.out_csv = f"data/multiband_{args.profile}_target.csv"
    if args.out_sidecar is None:
        args.out_sidecar = str(Path(args.out_csv).with_suffix(".json"))
    return args


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
    x_des, dx_des, quat_des, yaw_traj, roll_traj, tilt_traj = build_multiband_trajectory(
        t_s,
        x_anchor,
        q_anchor_xyzw,
        amp_x=args.amp_x,
        amp_y=args.amp_y,
        amp_z=args.amp_z,
        amp_yaw=float(args.amp_yaw),
        amp_roll=float(args.amp_roll),
        amp_ramp_s=float(args.amp_ramp),
        amp_rx=float(args.amp_rx),
        amp_ry=float(args.amp_ry),
        ramp_down_s=float(args.amp_ramp_down),
        profile=args.profile,
    )

    initial_offset = np.linalg.norm(x_des[0] - x_anchor)
    if initial_offset > 1e-6:
        raise ValueError(
            f"sanity check failed: |x_des(0) - x_anchor| = {initial_offset:.6e} m"
        )
    # quat_des(0) must equal q_anchor (within sign): yaw(0) = roll(0) = tilt(0) = 0
    # by the ramp envelope, so q_yaw_base = q_tilt_base = q_roll_local = identity.
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

    # Peak rate diagnostics for the commanded trajectory, reported for
    # inspection. Nothing caps them; robot.yaml's per-tick position clamp
    # (error_delta_pos) is what actually bounds the arm at runtime.
    peak_dx = float(np.max(np.abs(dx_des[:, 0])))
    peak_dy = float(np.max(np.abs(dx_des[:, 1])))
    peak_dz = float(np.max(np.abs(dx_des[:, 2])))
    peak_dyaw = float(np.max(np.abs(np.gradient(yaw_traj, dt))))
    peak_droll = float(np.max(np.abs(np.gradient(roll_traj, dt))))
    peak_drx, peak_dry = np.max(np.abs(np.gradient(tilt_traj, dt, axis=0)), axis=0).tolist()
    peak_speed_cart = float(np.max(np.linalg.norm(dx_des, axis=1)))
    # Largest angle between q_des and the anchor, measured on the quaternions
    # (the per-angle amplitudes do not add up: yaw and roll can cancel).
    peak_ori_offset = float(np.max(2.0 * np.arccos(np.clip(np.abs(quat_des @ q_anchor_xyzw), 0.0, 1.0))))

    # One entry per tone, three per axis.
    tones = multiband_tones(args.profile)
    pos_axes, ori_axes = ("x", "y", "z"), ("yaw", "roll", "rx", "ry")
    pos_freq_set = {axis: [f for f, _, _ in tones[axis]] for axis in pos_axes}
    pos_phase_set = {axis: [ph for _, _, ph in tones[axis]] for axis in pos_axes}
    ori_freq_set = {axis: [f for f, _, _ in tones[axis]] for axis in ori_axes}
    ori_phase_set = {axis: [ph for _, _, ph in tones[axis]] for axis in ori_axes}
    has_rotation = any(a > 0 for a in (args.amp_yaw, args.amp_roll, args.amp_rx, args.amp_ry))

    payload = {
        "schema_version": 2,
        "controller": "multiband_excitation_target" if has_rotation else "multiband_pos_only_target",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "target_csv_path": str(out_csv),
        "base_sidecar_path": str(base_sidecar),
        "hz": int(args.hz),
        "duration_s": float(args.duration),
        "num_samples": int(n),
        "profile": args.profile,
        "amp_ramp_s": float(args.amp_ramp),
        "ramp_down_s": float(args.amp_ramp_down),
        "amplitude_m": {"x": float(args.amp_x), "y": float(args.amp_y), "z": float(args.amp_z)},
        "amplitude_rad": {"yaw": float(args.amp_yaw), "roll": float(args.amp_roll),
                          "rx": float(args.amp_rx), "ry": float(args.amp_ry)},
        # Relative amplitude per tone, same order as freq_set_hz / ori_freq_set_hz.
        "tone_rel_amp": {axis: [a for _, a, _ in tones[axis]] for axis in pos_axes + ori_axes},
        "freq_set_hz": pos_freq_set,
        "phase_rad": pos_phase_set,
        "ori_freq_set_hz": ori_freq_set,
        "ori_phase_rad": ori_phase_set,
        "peak_rates": {
            "cart_speed_m_s": peak_speed_cart,
            "dx_m_s": [peak_dx, peak_dy, peak_dz],
            "dyaw_rad_s": peak_dyaw,
            "droll_rad_s": peak_droll,
            "drx_rad_s": peak_drx,
            "dry_rad_s": peak_dry,
            "max_ori_offset_rad": peak_ori_offset,
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

    print(f"[gen_excitation_traj] profile '{args.profile}': {len(tones['x'])} tones per axis, "
          f"fade-in {args.amp_ramp:g} s, fade-out {args.amp_ramp_down:g} s")
    print(f"[gen_excitation_traj] wrote {out_csv}")
    print(f"[gen_excitation_traj] wrote {out_sidecar}")
    print(
        f"[gen_excitation_traj] peak |dx_des| [m/s]: x={peak_dx:.4f}, y={peak_dy:.4f}, z={peak_dz:.4f}  "
        f"(|dx|_max={peak_speed_cart:.4f} m/s)"
    )
    print(
        "[gen_excitation_traj] peak rotation rates [rad/s]: "
        f"dyaw={peak_dyaw:.4f}, droll={peak_droll:.4f}, drx={peak_drx:.4f}, dry={peak_dry:.4f}. "
        f"yaw amp={args.amp_yaw:.3f} rad, roll amp={args.amp_roll:.3f} rad, "
        f"rx amp={args.amp_rx:.3f} rad, ry amp={args.amp_ry:.3f} rad. "
        f"q_des max ang-offset from anchor = {peak_ori_offset:.3f} rad"
    )
    return 0



if __name__ == "__main__":
    raise SystemExit(main())
