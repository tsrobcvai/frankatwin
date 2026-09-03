"""Cartesian task-impedance example.

Three trajectory modes are supported (selected via ``--mode``):

* ``sine`` (default, original behavior): single z-axis sinusoid around the
  pose captured at startup. Used as a smoke test for the PC ↔ NUC daemon
  pipeline (README "Verifying the full pipeline").

* ``multiband``: SysID v3 multi-band position excitation + optional base-yaw /
  EE-roll rotation excitation, reusing :func:`build_multiband_trajectory` from
  ``scripts/gen_excitation_traj.py`` (the same math that produced the SysID
  v3 training data). Drives the target via the 50 Hz Python control loop and
  optionally logs per-tick state to CSV + sidecar for the IsaacLab replay.

* ``chirp``: SysID v4 linear-chirp excitation (see ``scripts/gen_chirp_traj.py``).
  Linear frequency sweep f0->f1 across all 6 Cartesian DOFs (xyz + rx/ry/rz)
  with phase offsets at k*pi/3, asymmetric ramp.  Recommended companion gains
  are kp_pos=800, kp_ori=50 (stiff -- the chirp design assumes tight tracking
  so the recorded q is rich in high-freq content).

Prerequisites:
  * On the NUC: ``python -m frankatwin.daemon --config config/robot.yaml``
  * On this machine: ``pip install -e .`` (or PYTHONPATH=python)

Examples:
  # Smoke test (original sine):
  python examples/cart_impedance.py

  # v3 multiband excitation, write CSV + sidecar for IsaacLab replay:
  python examples/cart_impedance.py --mode multiband \\
      --amp-yaw 0.05 --amp-roll 0.05 --duration 12 \\
      --kp-pos 200 --kp-ori 20 \\
      --log data/multiband_$(date +%Y%m%d_%H%M%S).csv

  # Dry run (no robot connection), just inspect peak rates:
  python examples/cart_impedance.py --mode multiband --amp-yaw 0.05 \\
      --amp-roll 0.05 --dry-run
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from frankatwin.config import load_config
from frankatwin.remote_client import FrankaTwinClient

# `gen_excitation_traj` lives under scripts/ which is not a package. Add it to
# sys.path so we can import `build_multiband_trajectory` without copying code.
_SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "scripts"
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
from gen_excitation_traj import (  # noqa: E402
    ORI_FREQS,
    POS_FREQS,
    build_multiband_trajectory,
)
from gen_chirp_traj import (  # noqa: E402
    CHIRP_F0_DEFAULT,
    CHIRP_F1_DEFAULT,
    PHASE_OFFSETS,
    build_chirp_trajectory,
)

# Safety conventions shared with gen_excitation_traj.py / gen_chirp_traj.py.
# The Python loop runs at lower rate so
# tracking-error aborts on the NUC are the actual safety net, but we still
# pre-flight the *target* rates here so a bad CLI doesn't get sent to a robot.
CART_DX_PEAK_LIMIT_MPS = 0.30
ORI_DOT_PEAK_LIMIT_RPS = 0.50
ORI_TRACK_ABORT_RAD = 0.30


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--mode", choices=["sine", "multiband", "chirp"], default="sine",
                   help="Trajectory shape (default: sine — README smoke test).")
    p.add_argument("--config", type=str, default=None, help="Path to robot.yaml")
    p.add_argument("--duration", type=float, default=None,
                   help="Total run time [s]. Default: sine=4.0, multiband=12.0, chirp=8.0.")
    p.add_argument("--rate", type=float, default=50.0,
                   help="Python target update rate [Hz] (default: 50).")
    p.add_argument("--kp-pos", type=float, default=None,
                   help="Cartesian position stiffness (default: from robot.yaml).")
    p.add_argument("--kp-ori", type=float, default=None,
                   help="Cartesian orientation stiffness (default: from robot.yaml).")
    # osc_shm per-tick safety clamps.  Default None -> daemon keeps its
    # current value (which comes from robot.yaml at daemon startup).  Pass
    # explicitly to relax the clamps for aggressive trajectories (e.g. chirp
    # mode at full UR5e amps): the controller will then run despite large
    # tracking error instead of latching its tau output to zero.
    p.add_argument("--err-delta-pos", type=float, default=None,
                   help="Override osc_shm |e_pos|_inf abort threshold [m] (default: keep daemon value, typically 0.05).")
    p.add_argument("--err-delta-rot", type=float, default=None,
                   help="Override osc_shm |e_ori| abort threshold [rad] (default: keep daemon value, typically 0.30).")

    # sine-mode parameters
    p.add_argument("--amp", type=float, default=0.05,
                   help="[sine] z-axis amplitude [m] (default: 0.05).")
    p.add_argument("--freq", type=float, default=0.5,
                   help="[sine] sinusoid frequency [Hz] (default: 0.5).")

    # multiband / chirp shared position amplitudes.  Defaults resolve in main()
    # based on --mode: multiband gets (0.10, 0.10, 0.08), chirp gets (0.10, 0.10, 0.15).
    p.add_argument("--amp-x", type=float, default=None, help="X amplitude [m] (mode-dependent default).")
    p.add_argument("--amp-y", type=float, default=None, help="Y amplitude [m] (mode-dependent default).")
    p.add_argument("--amp-z", type=float, default=None, help="Z amplitude [m] (mode-dependent default).")
    p.add_argument("--amp-yaw", type=float, default=0.25,
                   help="[multiband] yaw (about world-z, drives j1) amplitude [rad] (default: 0.25).")
    p.add_argument("--amp-roll", type=float, default=0.20,
                   help="[multiband] roll (about EE-z, drives j5/j7) amplitude [rad] (default: 0.20).")
    p.add_argument("--high-band-ratio", type=float, default=0.20,
                   help="[multiband] high-band amplitude as fraction of low-band (default: 0.20).")
    p.add_argument("--amp-ramp", type=float, default=2.0,
                   help="[multiband] half-cosine envelope ramp length [s].")

    # chirp-mode parameters (v4) -- match UR5e collect_sysid_data shape exactly,
    # with f1 lowered (UR5e=3.0) to the Franka production default so |dx|_peak
    # stays ~0.46 m/s.
    p.add_argument("--f0", type=float, default=CHIRP_F0_DEFAULT,
                   help="[chirp] start frequency [Hz] (default: 0.1, UR5e).")
    p.add_argument("--f1", type=float, default=CHIRP_F1_DEFAULT,
                   help="[chirp] end frequency [Hz] (default: 0.7, lowered from UR5e 3.0).")
    p.add_argument("--amp-rx", type=float, default=0.50,
                   help="[chirp] world-x rotation amplitude [rad] (default: 0.50, UR5e).")
    p.add_argument("--amp-ry", type=float, default=0.25,
                   help="[chirp] world-y rotation amplitude [rad] (default: 0.25, UR5e; drives J6).")
    p.add_argument("--amp-rz", type=float, default=0.50,
                   help="[chirp] world-z rotation amplitude [rad] (default: 0.50, UR5e; drives J1).")
    p.add_argument("--ramp-up", type=float, default=2.0,
                   help="[chirp] linear ramp-up length [s] (default: 2.0, UR5e).")
    p.add_argument("--ramp-down", type=float, default=3.0,
                   help="[chirp] linear ramp-down length [s] (default: 3.0, UR5e).")

    # logging
    p.add_argument("--log", type=str, default=None,
                   help="CSV log path. If unset, no log is written.")
    p.add_argument("--sidecar", type=str, default=None,
                   help="Sidecar JSON path. Defaults to <log>.json.")
    p.add_argument("--dry-run", action="store_true",
                   help="Build trajectory and report peak rates; do NOT connect to robot.")
    return p.parse_args()


def _quat_wxyz_to_xyzw(q_wxyz: np.ndarray) -> np.ndarray:
    """Convert (..., 4) wxyz quaternion to xyzw."""
    q = np.asarray(q_wxyz, dtype=np.float64)
    return np.stack((q[..., 1], q[..., 2], q[..., 3], q[..., 0]), axis=-1)


def _quat_xyzw_to_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
    """Convert (..., 4) xyzw quaternion to wxyz."""
    q = np.asarray(q_xyzw, dtype=np.float64)
    return np.stack((q[..., 3], q[..., 0], q[..., 1], q[..., 2]), axis=-1)


def _build_trajectory(
    mode: str,
    t_grid: np.ndarray,
    x_anchor: np.ndarray,
    q_anchor_xyzw: np.ndarray,
    args: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return (x_des, dx_des, quat_des_xyzw, rot_a, rot_b) for the requested mode.

    For ``multiband``: rot_a = yaw, rot_b = roll (scalar per-tick).
    For ``chirp``:  rot_a = (rx, ry, rz) magnitude per-tick (axis-angle norm),
                    rot_b = zeros (kept for tuple compatibility).
    For ``sine``:   both zeros.
    """
    if mode == "multiband":
        x_des, dx_des, quat_des_xyzw, yaw, roll = build_multiband_trajectory(
            t_grid, x_anchor, q_anchor_xyzw,
            amp_x=args.amp_x, amp_y=args.amp_y, amp_z=args.amp_z,
            amp_yaw=args.amp_yaw, amp_roll=args.amp_roll,
            high_band_ratio=args.high_band_ratio,
            amp_ramp_s=args.amp_ramp,
        )
        return x_des, dx_des, quat_des_xyzw, yaw, roll

    if mode == "chirp":
        x_des, dx_des, quat_des_xyzw, rot_offsets = build_chirp_trajectory(
            t_grid, x_anchor, q_anchor_xyzw,
            f0=args.f0, f1=args.f1,
            amp_x=args.amp_x, amp_y=args.amp_y, amp_z=args.amp_z,
            amp_rx=args.amp_rx, amp_ry=args.amp_ry, amp_rz=args.amp_rz,
            ramp_up_s=args.ramp_up, ramp_down_s=args.ramp_down,
        )
        # rot_offsets is (T, 3); flatten to a magnitude for the legacy
        # (yaw, roll) interface so the rest of the pipeline keeps working.
        # _print_peak_rates is patched separately to inspect the full vector.
        rot_a = np.linalg.norm(rot_offsets, axis=1)
        rot_b = np.zeros_like(rot_a)
        # Stash the full (T,3) on args so the peak-rate / sidecar paths can
        # report per-axis numbers without re-running the builder.
        args._chirp_rot_offsets = rot_offsets  # type: ignore[attr-defined]
        return x_des, dx_des, quat_des_xyzw, rot_a, rot_b

    # sine mode: single z-axis sinusoid around the anchor, orientation held.
    omega = 2.0 * math.pi * args.freq
    z_off = args.amp * np.sin(omega * t_grid)
    dz_off = args.amp * omega * np.cos(omega * t_grid)
    x_des = x_anchor[None, :] + np.column_stack(
        (np.zeros_like(t_grid), np.zeros_like(t_grid), z_off)
    )
    dx_des = np.column_stack(
        (np.zeros_like(t_grid), np.zeros_like(t_grid), dz_off)
    )
    quat_des_xyzw = np.broadcast_to(q_anchor_xyzw, (t_grid.shape[0], 4)).copy()
    yaw = np.zeros_like(t_grid)
    roll = np.zeros_like(t_grid)
    return x_des, dx_des, quat_des_xyzw, yaw, roll


def _print_peak_rates(
    dx_des: np.ndarray,
    rot_a: np.ndarray,
    rot_b: np.ndarray,
    args: argparse.Namespace,
) -> dict:
    """Print and return peak-rate diagnostics; warn if safety conventions exceeded.

    ``rot_a`` / ``rot_b`` interpretation depends on mode (see _build_trajectory).
    """
    dt_grid = 1.0 / args.rate
    peak_dx, peak_dy, peak_dz = np.max(np.abs(dx_des), axis=0).tolist()
    peak_cart_speed = float(np.max(np.linalg.norm(dx_des, axis=1)))

    if args.mode == "chirp":
        # rot_a stores |rot_offset|; the per-axis vectors live on args.
        rot_offsets = getattr(args, "_chirp_rot_offsets", None)
        if rot_offsets is None or rot_offsets.size == 0:
            peak_drx = peak_dry = peak_drz = 0.0
            peak_ori_offset = 0.0
        else:
            drot = np.gradient(rot_offsets, dt_grid, axis=0)
            peak_drx, peak_dry, peak_drz = np.max(np.abs(drot), axis=0).tolist()
            peak_ori_offset = float(np.max(np.linalg.norm(rot_offsets, axis=1)))
        peak_ang_rate = float(max(peak_drx, peak_dry, peak_drz))
        print(
            f"[cart_impedance] peak |dx_des| [m/s]: x={peak_dx:.4f}, y={peak_dy:.4f}, "
            f"z={peak_dz:.4f}  (|dx|_max={peak_cart_speed:.4f}, convention {CART_DX_PEAK_LIMIT_MPS:.2f})"
        )
        print(
            f"[cart_impedance] peak rotation rates [rad/s]: drx={peak_drx:.4f}, "
            f"dry={peak_dry:.4f}, drz={peak_drz:.4f}  (convention {ORI_DOT_PEAK_LIMIT_RPS:.2f}). "
            f"max |rot_offset| ~ {peak_ori_offset:.3f} rad (abort {ORI_TRACK_ABORT_RAD:.2f})"
        )
        if peak_cart_speed > CART_DX_PEAK_LIMIT_MPS:
            print(
                f"[cart_impedance] WARNING: peak Cartesian speed {peak_cart_speed:.3f} > "
                f"{CART_DX_PEAK_LIMIT_MPS:.2f} m/s convention. Reduce --amp-x/y/z or --f1.",
                file=sys.stderr,
            )
        if peak_ang_rate > ORI_DOT_PEAK_LIMIT_RPS:
            print(
                f"[cart_impedance] WARNING: peak angular rate {peak_ang_rate:.3f} > "
                f"{ORI_DOT_PEAK_LIMIT_RPS:.2f} rad/s convention.",
                file=sys.stderr,
            )
        if peak_ori_offset > 0.8 * ORI_TRACK_ABORT_RAD:
            print(
                f"[cart_impedance] WARNING: max |rot_offset| = {peak_ori_offset:.3f} rad "
                f"is > 80% of {ORI_TRACK_ABORT_RAD:.2f} rad runtime abort.",
                file=sys.stderr,
            )
        return {
            "cart_speed_m_s": peak_cart_speed,
            "dx_m_s": [peak_dx, peak_dy, peak_dz],
            "drot_rad_s": [peak_drx, peak_dry, peak_drz],
            "max_ori_offset_rad": peak_ori_offset,
        }

    # --- multiband / sine path (legacy interface: rot_a=yaw, rot_b=roll) ----
    yaw, roll = rot_a, rot_b
    peak_dyaw = float(np.max(np.abs(np.gradient(yaw, dt_grid)))) if yaw.size > 1 else 0.0
    peak_droll = float(np.max(np.abs(np.gradient(roll, dt_grid)))) if roll.size > 1 else 0.0
    peak_ori_offset = float(args.amp_yaw + args.amp_roll) * (1.0 + float(args.high_band_ratio))

    print(
        f"[cart_impedance] peak |dx_des| [m/s]: x={peak_dx:.4f}, y={peak_dy:.4f}, "
        f"z={peak_dz:.4f}  (|dx|_max={peak_cart_speed:.4f}, convention {CART_DX_PEAK_LIMIT_MPS:.2f})"
    )
    print(
        f"[cart_impedance] peak rotation rates [rad/s]: dyaw={peak_dyaw:.4f}, "
        f"droll={peak_droll:.4f}  (convention {ORI_DOT_PEAK_LIMIT_RPS:.2f}). "
        f"q_des max ang-offset ~ {peak_ori_offset:.3f} rad "
        f"(abort {ORI_TRACK_ABORT_RAD:.2f})"
    )
    if peak_cart_speed > CART_DX_PEAK_LIMIT_MPS:
        print(
            f"[cart_impedance] WARNING: peak Cartesian speed {peak_cart_speed:.3f} > "
            f"{CART_DX_PEAK_LIMIT_MPS:.2f} m/s convention. Reduce --amp-* or --high-band-ratio.",
            file=sys.stderr,
        )
    if max(peak_dyaw, peak_droll) > ORI_DOT_PEAK_LIMIT_RPS:
        print(
            f"[cart_impedance] WARNING: peak angular rate {max(peak_dyaw, peak_droll):.3f} > "
            f"{ORI_DOT_PEAK_LIMIT_RPS:.2f} rad/s convention.",
            file=sys.stderr,
        )
    if peak_ori_offset > 0.8 * ORI_TRACK_ABORT_RAD:
        print(
            f"[cart_impedance] WARNING: amp_yaw + amp_roll = {peak_ori_offset:.3f} rad is > 80% "
            f"of {ORI_TRACK_ABORT_RAD:.2f} rad runtime abort.",
            file=sys.stderr,
        )
    return {
        "cart_speed_m_s": peak_cart_speed,
        "dx_m_s": [peak_dx, peak_dy, peak_dz],
        "dyaw_rad_s": peak_dyaw,
        "droll_rad_s": peak_droll,
    }


def _write_csv(
    path: Path,
    t_grid: np.ndarray,
    x_des: np.ndarray,
    dx_des: np.ndarray,
    quat_des_xyzw: np.ndarray,
    q: np.ndarray,
    dq: np.ndarray,
    x_act: np.ndarray,
    quat_act_xyzw: np.ndarray,
    period_ms: np.ndarray,
    tau_cmd: np.ndarray,
    tau_J: np.ndarray,
) -> None:
    columns = ["t_s", "period_ms"]
    columns += [f"q{i}" for i in range(1, 8)]
    columns += [f"dq{i}" for i in range(1, 8)]
    columns += ["x_x", "x_y", "x_z"]
    columns += ["quat_x", "quat_y", "quat_z", "quat_w"]
    columns += ["x_des_x", "x_des_y", "x_des_z"]
    columns += ["dx_des_x", "dx_des_y", "dx_des_z"]
    columns += ["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]
    # tau = commanded impedance torque (gravity excluded); tau_J = measured
    # link-side torque (gravity included, shm v3+; NaN from older daemons).
    columns += [f"tau{i}" for i in range(1, 8)]
    columns += [f"tau_J{i}" for i in range(1, 8)]
    data = np.column_stack(
        (t_grid, period_ms, q, dq, x_act, quat_act_xyzw,
         x_des, dx_des, quat_des_xyzw, tau_cmd, tau_J)
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, data, delimiter=",", header=",".join(columns), comments="", fmt="%.9f")
    print(f"[cart_impedance] wrote CSV: {path}")


def _write_sidecar(
    path: Path,
    args: argparse.Namespace,
    q_init: np.ndarray,
    x_anchor: np.ndarray,
    q_anchor_xyzw: np.ndarray,
    duration_s: float,
    num_samples: int,
    csv_path: Path | None,
    peak_rates: dict,
    kp_pos: float,
    kp_ori: float,
    summary: dict | None,
    abort: dict,
) -> None:
    if args.mode == "multiband":
        pos_freq_set = {axis: list(POS_FREQS[axis][:2]) for axis in ("x", "y", "z")}
        pos_phase_set = {axis: [POS_FREQS[axis][2], 0.0] for axis in ("x", "y", "z")}
        ori_freq_set = {axis: list(ORI_FREQS[axis][:2]) for axis in ("yaw", "roll")}
        ori_phase_set = {axis: [ORI_FREQS[axis][2], 0.0] for axis in ("yaw", "roll")}
        controller_name = "python_multiband_excitation"
    elif args.mode == "chirp":
        # Linear chirp: report (f0, f1) per axis and per-axis phase offsets.
        axes = ["x", "y", "z", "rx", "ry", "rz"]
        pos_freq_set = {axes[i]: [float(args.f0), float(args.f1)] for i in range(3)}
        pos_phase_set = {axes[i]: [float(PHASE_OFFSETS[i]), 0.0] for i in range(3)}
        ori_freq_set = {axes[i]: [float(args.f0), float(args.f1)] for i in range(3, 6)}
        ori_phase_set = {axes[i]: [float(PHASE_OFFSETS[i]), 0.0] for i in range(3, 6)}
        controller_name = "python_v4_chirp_excitation"
    else:
        pos_freq_set = {"z": [args.freq, 0.0]}
        pos_phase_set = {"z": [0.0, 0.0]}
        ori_freq_set = {}
        ori_phase_set = {}
        controller_name = "python_sine"

    if args.mode == "multiband":
        amplitude_m = {"x": float(args.amp_x), "y": float(args.amp_y), "z": float(args.amp_z)}
        amplitude_rad = {"yaw": float(args.amp_yaw), "roll": float(args.amp_roll)}
    elif args.mode == "chirp":
        amplitude_m = {"x": float(args.amp_x), "y": float(args.amp_y), "z": float(args.amp_z)}
        amplitude_rad = {"rx": float(args.amp_rx), "ry": float(args.amp_ry), "rz": float(args.amp_rz)}
    else:
        amplitude_m = {"x": 0.0, "y": 0.0, "z": float(args.amp)}
        amplitude_rad = {"yaw": 0.0, "roll": 0.0}

    payload = {
        "schema_version": 2,
        "controller": controller_name,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "csv_path": str(csv_path) if csv_path is not None else None,
        "sidecar_path": str(path),
        "control_rate_hz": float(args.rate),
        "duration_s": float(duration_s),
        "num_samples": int(num_samples),
        "amp_ramp_s": float(args.amp_ramp) if args.mode == "multiband" else 0.0,
        "high_band_ratio": float(args.high_band_ratio) if args.mode == "multiband" else 0.0,
        "amplitude_m": amplitude_m,
        "amplitude_rad": amplitude_rad,
        "freq_set_hz": pos_freq_set,
        "phase_rad": pos_phase_set,
        "ori_freq_set_hz": ori_freq_set,
        "ori_phase_rad": ori_phase_set,
        "peak_rates": peak_rates,
        "q_init": q_init.tolist(),
        "x_anchor": x_anchor.tolist(),
        "q_anchor_xyzw": q_anchor_xyzw.tolist(),
        "args": {
            "kp_pos": float(kp_pos),
            "kp_ori": float(kp_ori),
            "kd_pos": 2.0 * math.sqrt(float(kp_pos)),
            "kd_ori": 2.0 * math.sqrt(float(kp_ori)),
            "duration": float(duration_s),
            "rate": float(args.rate),
            "mode": args.mode,
            "err_delta_pos_override": args.err_delta_pos,
            "err_delta_rot_override": args.err_delta_rot,
            **({"f0_hz": float(args.f0), "f1_hz": float(args.f1),
                "ramp_up_s": float(args.ramp_up), "ramp_down_s": float(args.ramp_down)}
                if args.mode == "chirp" else {}),
        },
        "abort": abort,
        "summary": summary,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"[cart_impedance] wrote sidecar: {path}")


def _compute_summary(
    x_des: np.ndarray,
    x_act: np.ndarray,
    quat_des_xyzw: np.ndarray,
    quat_act_xyzw: np.ndarray,
) -> dict:
    e_pos = x_des - x_act
    # Shortest-path orientation angle in rad.
    dot = np.clip(np.abs(np.einsum("ij,ij->i", quat_des_xyzw, quat_act_xyzw)), 0.0, 1.0)
    theta = 2.0 * np.arccos(dot)
    return {
        "ticks": int(x_des.shape[0]),
        "err_pos_rms_xyz": np.sqrt(np.mean(e_pos**2, axis=0)).tolist(),
        "err_pos_abs_max_xyz": np.max(np.abs(e_pos), axis=0).tolist(),
        "err_pos_inf_max": float(np.max(np.linalg.norm(e_pos, axis=1))),
        "err_ori_rms_rad": float(np.sqrt(np.mean(theta**2))),
        "err_ori_max_rad": float(np.max(theta)),
    }


def main() -> int:
    args = parse_args()
    if args.duration is None:
        # chirp (v4) runs 8.0 s; multiband (v3) runs 12.0 s.
        args.duration = 8.0 if args.mode == "chirp" else (12.0 if args.mode == "multiband" else 4.0)
    # Resolve mode-dependent amplitude defaults.
    # multiband v3: 0.10/0.10/0.08 (the 2026-05-25 v3 collection).
    # chirp v4: UR5e-exact magnitudes (0.10/0.10/0.15, Z = 1.5x XY).
    if args.mode == "chirp":
        if args.amp_x is None: args.amp_x = 0.10
        if args.amp_y is None: args.amp_y = 0.10
        if args.amp_z is None: args.amp_z = 0.15
    else:
        if args.amp_x is None: args.amp_x = 0.10
        if args.amp_y is None: args.amp_y = 0.10
        if args.amp_z is None: args.amp_z = 0.08
    if args.log is None and not args.dry_run:
        print("[cart_impedance] (no --log given, state will not be saved to disk)", file=sys.stderr)

    log_path = Path(args.log).expanduser().resolve() if args.log else None
    sidecar_path: Path | None
    if log_path is not None:
        sidecar_path = (
            Path(args.sidecar).expanduser().resolve()
            if args.sidecar
            else log_path.with_suffix(".json")
        )
    else:
        sidecar_path = (
            Path(args.sidecar).expanduser().resolve() if args.sidecar else None
        )

    dt_grid = 1.0 / float(args.rate)
    n = int(round(float(args.duration) * float(args.rate)))
    t_grid = np.arange(n, dtype=np.float64) * dt_grid

    # ----------------------------------------------------------- dry-run path
    if args.dry_run:
        # Use a synthetic anchor for the dry run so we can still inspect the
        # trajectory shape without robot access.
        x_anchor = np.array([0.5, 0.0, 0.4], dtype=np.float64)
        q_anchor_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        x_des, dx_des, quat_des_xyzw, yaw, roll = _build_trajectory(
            args.mode, t_grid, x_anchor, q_anchor_xyzw, args
        )
        _print_peak_rates(dx_des, yaw, roll, args)
        print(f"[cart_impedance] dry-run: built {n} samples over {args.duration:.2f} s "
              f"@ {args.rate:.1f} Hz; first x_des={x_des[0]}, last={x_des[-1]}")
        return 0

    # ----------------------------------------------------------- live path
    cfg = load_config(args.config)
    kp_pos = float(args.kp_pos) if args.kp_pos is not None else float(cfg.control.kp_pos)
    kp_ori = float(args.kp_ori) if args.kp_ori is not None else float(cfg.control.kp_ori)

    with FrankaTwinClient(cfg) as robot:
        robot.set_gains(
            kp_pos=kp_pos,
            kp_ori=kp_ori,
            error_delta_pos=args.err_delta_pos,
            error_delta_rot=args.err_delta_rot,
        )
        if args.err_delta_pos is not None or args.err_delta_rot is not None:
            print(
                f"[cart_impedance] osc_shm clamps overridden: "
                f"err_delta_pos={args.err_delta_pos}, err_delta_rot={args.err_delta_rot}"
            )
        state = robot.wait_for_state(timeout_s=3.0)
        q_init = state.q.copy()
        x_anchor = state.ee_pos.copy()
        anchor_quat_wxyz = state.ee_quat.copy()
        q_anchor_xyzw = _quat_wxyz_to_xyzw(anchor_quat_wxyz)
        q_anchor_xyzw = q_anchor_xyzw / max(float(np.linalg.norm(q_anchor_xyzw)), 1e-12)
        print(f"[cart_impedance] anchor x = {x_anchor}")
        print(f"[cart_impedance] anchor quat (xyzw) = {q_anchor_xyzw}")

        x_des, dx_des, quat_des_xyzw, yaw, roll = _build_trajectory(
            args.mode, t_grid, x_anchor, q_anchor_xyzw, args
        )
        peak_rates = _print_peak_rates(dx_des, yaw, roll, args)

        # Logging buffers (allocated unconditionally so the summary always runs).
        log_period = np.zeros(n, dtype=np.float64)
        log_q = np.zeros((n, 7), dtype=np.float64)
        log_dq = np.zeros((n, 7), dtype=np.float64)
        log_x = np.zeros((n, 3), dtype=np.float64)
        log_quat_xyzw = np.zeros((n, 4), dtype=np.float64)
        log_tau = np.zeros((n, 7), dtype=np.float64)
        log_tau_J = np.full((n, 7), np.nan, dtype=np.float64)

        abort = {"code": 0, "name": "none", "value": 0.0, "time_s": 0.0}
        summary: dict | None = None

        # Convert quat_des in-place to wxyz for the send path (avoids re-stacking
        # per-tick). Keep xyzw view too for logging.
        quat_des_wxyz = _quat_xyzw_to_wxyz(quat_des_xyzw)

        last_state_seq = -1
        t0 = time.monotonic()
        last_t = t0
        try:
            for i in range(n):
                robot.set_ee_target(x_des[i], quat_des_wxyz[i])
                fresh = robot.get_state()
                now = time.monotonic()
                log_period[i] = (now - last_t) * 1000.0
                last_t = now
                if fresh is not None:
                    log_q[i] = fresh.q
                    log_dq[i] = fresh.dq
                    log_x[i] = fresh.ee_pos
                    log_quat_xyzw[i] = _quat_wxyz_to_xyzw(fresh.ee_quat)
                    log_tau[i] = fresh.tau
                    log_tau_J[i] = fresh.tau_J
                    last_state_seq = fresh.seq
                else:
                    # No frame yet — repeat last row (will be zeros only at i=0).
                    if i > 0:
                        log_q[i] = log_q[i - 1]
                        log_dq[i] = log_dq[i - 1]
                        log_x[i] = log_x[i - 1]
                        log_quat_xyzw[i] = log_quat_xyzw[i - 1]
                        log_tau[i] = log_tau[i - 1]
                        log_tau_J[i] = log_tau_J[i - 1]
                sleep_for = (t0 + (i + 1) * dt_grid) - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)

            # Return to anchor and let the impedance settle.
            robot.set_ee_target(x_anchor, anchor_quat_wxyz)
            time.sleep(0.5)
            final = robot.get_state()
            if final is not None:
                err = float(np.linalg.norm(final.ee_pos - x_anchor))
                print(f"[cart_impedance] final |ee_pos - anchor| = {err:.4f} m")

            summary = _compute_summary(x_des, log_x, quat_des_xyzw, log_quat_xyzw)
            print(
                f"[cart_impedance] tracking err_pos_rms_xyz [mm] = "
                f"{[round(v * 1000.0, 3) for v in summary['err_pos_rms_xyz']]}, "
                f"err_ori_rms = {summary['err_ori_rms_rad'] * 1000.0:.3f} mrad"
            )
            # Joint-torque headroom vs the Panda actuator limits (tau_J is the
            # measured link-side torque, gravity included).
            if np.isfinite(log_tau_J).any():
                tau_limits = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
                tau_abs_max = np.nanmax(np.abs(log_tau_J), axis=0)
                frac = tau_abs_max / tau_limits
                summary["tau_J_abs_max_nm"] = tau_abs_max.tolist()
                summary["tau_J_limit_frac"] = frac.tolist()
                summary["tau_cmd_abs_max_nm"] = np.max(np.abs(log_tau), axis=0).tolist()
                print(
                    "[cart_impedance] max |tau_J| [Nm] = "
                    + ", ".join(f"j{j+1}={tau_abs_max[j]:.2f} ({100.0*frac[j]:.0f}%)"
                                for j in range(7))
                    + "  (limits 87/87/87/87/12/12/12)"
                )
                if np.any(frac > 0.8):
                    worst = int(np.argmax(frac))
                    print(
                        f"[cart_impedance] WARNING: joint {worst+1} reached "
                        f"{100.0*frac[worst]:.0f}% of its torque limit",
                        file=sys.stderr,
                    )
            else:
                print("[cart_impedance] WARNING: no tau_J in state stream "
                      "(daemon predates shm v3?)", file=sys.stderr)
            if last_state_seq < 0:
                print("[cart_impedance] WARNING: no state frames received from daemon",
                      file=sys.stderr)
        except KeyboardInterrupt:
            abort = {"code": 1, "name": "keyboard_interrupt", "value": 0.0,
                     "time_s": float(time.monotonic() - t0)}
            print("[cart_impedance] interrupted by user", file=sys.stderr)

        if log_path is not None:
            _write_csv(
                log_path, t_grid, x_des, dx_des, quat_des_xyzw,
                log_q, log_dq, log_x, log_quat_xyzw, log_period,
                log_tau, log_tau_J,
            )
        if sidecar_path is not None:
            _write_sidecar(
                sidecar_path, args, q_init, x_anchor, q_anchor_xyzw,
                duration_s=float(args.duration), num_samples=n,
                csv_path=log_path, peak_rates=peak_rates,
                kp_pos=kp_pos, kp_ori=kp_ori,
                summary=summary, abort=abort,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
