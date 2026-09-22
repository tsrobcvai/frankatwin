"""Cartesian task-impedance example.

Three trajectory modes are supported (selected via ``--mode``):

* ``sine`` (default, original behavior): single z-axis sinusoid around the
  pose captured at startup. Used as a smoke test for the PC ↔ NUC daemon
  pipeline (README "Verifying the full pipeline").

* ``multiband``: stationary sinusoids summed per axis, reusing
  :func:`build_multiband_trajectory` from ``frankatwin.excitation.multiband``.
  The 6-DOF held-out validation run for a chirp fit: tilt about world x / y
  plus yaw, three bands up to 2 Hz with amplitudes ~ 0.35/f, and a fade-out
  back to the start pose (``--profile heldout``, the only profile). Drives the
  target via the 50 Hz Python control loop and optionally logs per-tick state
  to CSV + sidecar for the IsaacLab replay.

* ``chirp``: linear-chirp excitation (see ``frankatwin.excitation.chirp``).
  Linear frequency sweep across all 6 Cartesian DOFs (xyz + rx/ry/rz) with
  phase offsets at k*pi/3 and an asymmetric linear ramp, 8 s. Two bands
  (``--band``): ``low`` sweeps 0.1 -> 0.7 Hz at constant amplitude, ``high``
  sweeps 0.7 -> 3 Hz with the amplitude tapered as (0.7/f)^0.5 and a 1 s
  fade-out, sized for the fit gains kp 200/20 from a run on the arm (see
  chirp.py): predicted ~33 N of peak EE force against ~13 N for the low band --
  watch the torque line, and use --amp-taper-exp 2 at stiffer gains. The low
  band is the fit run, the high band an optional held-out run; gains per
  docs/sysid.md (200/20 for both, 500/30 for the multiband validation run).

Logging: ``--log-1khz PATH`` alone is enough for a sysid run. The daemon
records every controller tick in memory on the NUC; after the run the rows are
fetched and written on this machine at PATH, with ``<stem>_targets.csv`` and the
run's JSON sidecar (``<stem>.json``) next to it. Nothing is written on the NUC.
``--log`` adds the 50 Hz CSV of the state stream; it is optional.

Prerequisites:
  * On the NUC: ``python -m frankatwin.daemon``
  * On this machine: ``pip install -e .``
  * ``python examples/move_to.py`` before every run: the reference is anchored at
    the pose the arm is in, and a run leaves the joints away from where it
    started (the controller has no posture term), so runs collected back to
    back without it start from different configurations.

Examples:
  # Smoke test (original sine):
  python examples/cart_impedance.py

  # Held-out validation run for a chirp fit (6-DOF, three bands, fades out):
  python examples/cart_impedance.py --mode multiband --profile heldout \\
      --kp-pos 500 --kp-ori 30 --log data/sysid/multiband_heldout.csv

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
from frankatwin.excitation import (
    CHIRP_BANDS,
    MULTIBAND_PROFILES,
    PHASE_OFFSETS,
    build_chirp_trajectory,
    build_multiband_trajectory,
    multiband_tones,
)
from frankatwin.remote_client import FrankaTwinClient
from frankatwin.ring_log import targets_path_for

# Peak target rates are reported, not policed. The old 0.30 m/s / 0.50 rad/s
# pair came from the UR5e pipeline this excitation code was ported from, not
# from Franka -- libfranka's own ceilings are 3.0 m/s and 2.5 rad/s (see
# franka/rate_limiting.h) -- and the old multiband defaults sat just above the
# translational one, so the default run warned about itself. The safety net is
# osc_shm's tracking-error clamp on the NUC.


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--mode", choices=["sine", "multiband", "chirp"], default="sine",
                   help="Trajectory shape (default: sine — README smoke test).")
    p.add_argument("--config", type=str, default=None, help="Path to robot.yaml")
    p.add_argument("--duration", type=float, default=None,
                   help="Total run time [s]. Default: sine=4.0, multiband=12.0, chirp=8.0.")
    p.add_argument("--log-1khz", type=str, default=None,
                   help="Record a 1 kHz ring log (frankatwin.ring_log) and write it at this path on "
                        "this machine: every controller tick, setpoints stamped by the tick they took "
                        "effect, the --log columns plus seq/target_idx. The daemon records in memory "
                        "on the NUC and the rows are fetched after the run; nothing is written there. "
                        "Enough on its own for a sysid run: <stem>_targets.csv and the sidecar "
                        "(<stem>.json) land next to it. The setpoint rate stays --rate.")
    p.add_argument("--rate", type=float, default=50.0,
                   help="Python target update rate [Hz] (default: 50).")
    p.add_argument("--kp-pos", type=float, default=None,
                   help="Cartesian position stiffness (default: from robot.yaml).")
    p.add_argument("--kp-ori", type=float, default=None,
                   help="Cartesian orientation stiffness (default: from robot.yaml).")
    # osc_shm per-tick position safety clamp.  Default None -> daemon keeps
    # its current value (which comes from robot.yaml at daemon startup).  Pass
    # explicitly to relax it for aggressive trajectories (e.g. chirp mode at
    # full UR5e amps): the controller will then run despite large tracking
    # error instead of latching its tau output to zero.  The orientation
    # channel is pure impedance and has no clamp to relax.
    p.add_argument("--err-delta-pos", type=float, default=None,
                   help="Override osc_shm's position error clamp [m]: bounds the push to "
                        "kp_pos*this AND aborts past it. 0 = pure impedance. "
                        "Default: keep the daemon's value (robot.yaml, 0.0).")

    # sine-mode parameters
    p.add_argument("--amp", type=float, default=0.05,
                   help="[sine] z-axis amplitude [m] (default: 0.05).")
    p.add_argument("--freq", type=float, default=0.5,
                   help="[sine] sinusoid frequency [Hz] (default: 0.5).")

    # multiband / chirp shared position amplitudes.  Defaults resolve in main()
    # based on --mode: multiband gets its --profile's (0.06 on all three),
    # chirp gets (0.10, 0.10, 0.15).
    p.add_argument("--amp-x", type=float, default=None, help="X amplitude [m] (mode-dependent default).")
    p.add_argument("--amp-y", type=float, default=None, help="Y amplitude [m] (mode-dependent default).")
    p.add_argument("--amp-z", type=float, default=None, help="Z amplitude [m] (mode-dependent default).")
    # multiband-mode parameters. --profile sets the tone table and the defaults
    # of every flag below (MULTIBAND_PROFILES); explicit flags win.
    p.add_argument("--profile", choices=sorted(MULTIBAND_PROFILES), default="heldout",
                   help="[multiband] heldout = 6-DOF held-out validation run for a chirp fit: "
                        "yaw + tilt (rx, ry), three bands up to 2 Hz at amplitudes ~ 0.35/f, fades "
                        "out to the start pose (default and only profile: heldout).")
    p.add_argument("--amp-yaw", type=float, default=None,
                   help="[multiband] yaw (about world-z) amplitude [rad] (default: 0.25).")
    p.add_argument("--amp-roll", type=float, default=None,
                   help="[multiband] roll (about EE-z) amplitude [rad]. With the tool pointing down "
                        "this is the same rotation as yaw, reversed (default: 0).")
    p.add_argument("--amp-ramp", type=float, default=None,
                   help="[multiband] half-cosine fade-in length [s] (default: 2.0).")
    p.add_argument("--amp-ramp-down", type=float, default=None,
                   help="[multiband] half-cosine fade-out length [s], 0 = end at full amplitude "
                        "(default: 2.0).")

    # chirp-mode parameters -- match UR5e collect_sysid_data shape exactly.
    # --band sets f0 / f1 / the amplitude taper: low = 0.1->0.7 Hz at constant
    # amplitude (f1 lowered from UR5e's 3.0 so |dx|_peak stays ~0.46 m/s),
    # high = 0.7->3 Hz with amplitude ~ (0.7/f)^0.5 and a 1 s fade-out (sized
    # for kp 200/20). Explicit --f0 / --f1 / --amp-taper-exp / --ramp-* win.
    p.add_argument("--band", choices=sorted(CHIRP_BANDS), default="low",
                   help="[chirp] sweep band: low = 0.1->0.7 Hz; high = 0.7->3 Hz with a (0.7/f)^0.5 "
                        "amplitude taper and a 1 s fade-out, sized for kp 200/20 (predicted ~33 N "
                        "peak EE force vs ~13 N for low). Both 8 s; low is the fit run, high an "
                        "optional held-out run (default: low).")
    p.add_argument("--f0", type=float, default=None,
                   help="[chirp] start frequency [Hz] (default: the band's).")
    p.add_argument("--f1", type=float, default=None,
                   help="[chirp] end frequency [Hz] (default: the band's).")
    p.add_argument("--amp-taper-exp", type=float, default=None,
                   help="[chirp] amplitude taper exponent p, amplitudes scale as (f0/f)^p along the "
                        "sweep: 0 = constant, 1 = constant reference velocity, 2 = constant reference "
                        "acceleration (default: the band's; use 2 for the high band above kp 200/20).")
    p.add_argument("--amp-rx", type=float, default=None,
                   help="[chirp, multiband] world-x rotation (tilt) amplitude [rad] (default: chirp 0.50, "
                        "UR5e; multiband 0.20).")
    p.add_argument("--amp-ry", type=float, default=None,
                   help="[chirp, multiband] world-y rotation (tilt) amplitude [rad] (default: chirp 0.25, "
                        "UR5e, drives J6; multiband 0.15).")
    p.add_argument("--amp-rz", type=float, default=0.50,
                   help="[chirp] world-z rotation amplitude [rad] (default: 0.50, UR5e; drives J1).")
    p.add_argument("--ramp-up", type=float, default=None,
                   help="[chirp] linear ramp-up length [s] (default: the band's, 2.0 -- UR5e).")
    p.add_argument("--ramp-down", type=float, default=None,
                   help="[chirp] linear ramp-down length [s] (default: the band's; low 3.0 -- UR5e, "
                        "high 1.0).")

    # logging
    p.add_argument("--log", type=str, default=None,
                   help="Optional 50 Hz CSV of the state stream (the rows are the newest 100 Hz "
                        "frame at each setpoint; use --log-1khz for sysid).")
    p.add_argument("--sidecar", type=str, default=None,
                   help="Sidecar JSON path. Defaults to <log>.json, else to <log-1khz>.json. With "
                        "--log-1khz a sidecar is written as <log-1khz>.json in any case.")
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

    For ``multiband``: rot_a = yaw, rot_b = roll (scalar per-tick); the (T, 2)
                    tilt (rx, ry) is stashed on ``args``.
    For ``chirp``:  rot_a = (rx, ry, rz) magnitude per-tick (axis-angle norm),
                    rot_b = zeros (kept for tuple compatibility).
    For ``sine``:   both zeros.
    """
    if mode == "multiband":
        x_des, dx_des, quat_des_xyzw, yaw, roll, tilt = build_multiband_trajectory(
            t_grid, x_anchor, q_anchor_xyzw,
            amp_x=args.amp_x, amp_y=args.amp_y, amp_z=args.amp_z,
            amp_yaw=args.amp_yaw, amp_roll=args.amp_roll,
            amp_ramp_s=args.amp_ramp,
            amp_rx=args.amp_rx, amp_ry=args.amp_ry,
            ramp_down_s=args.amp_ramp_down,
            profile=args.profile,
        )
        args._multiband_tilt = tilt  # type: ignore[attr-defined]
        return x_des, dx_des, quat_des_xyzw, yaw, roll

    if mode == "chirp":
        x_des, dx_des, quat_des_xyzw, rot_offsets = build_chirp_trajectory(
            t_grid, x_anchor, q_anchor_xyzw,
            f0=args.f0, f1=args.f1,
            amp_x=args.amp_x, amp_y=args.amp_y, amp_z=args.amp_z,
            amp_rx=args.amp_rx, amp_ry=args.amp_ry, amp_rz=args.amp_rz,
            ramp_up_s=args.ramp_up, ramp_down_s=args.ramp_down,
            amp_taper_exp=args.amp_taper_exp,
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
    quat_des_xyzw: np.ndarray,
    q_anchor_xyzw: np.ndarray,
    rot_a: np.ndarray,
    rot_b: np.ndarray,
    args: argparse.Namespace,
) -> dict:
    """Print and return peak-rate diagnostics for the commanded trajectory.

    These are properties of the *reference*: ``dx_des`` is its analytic
    derivative, so they say how fast the target moves, not how fast the arm
    does -- under impedance control it lags behind.

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
            f"z={peak_dz:.4f}  (|dx|_max={peak_cart_speed:.4f})"
        )
        print(
            f"[cart_impedance] peak rotation rates [rad/s]: drx={peak_drx:.4f}, "
            f"dry={peak_dry:.4f}, drz={peak_drz:.4f}. "
            f"max |rot_offset| ~ {peak_ori_offset:.3f} rad"
        )
        return {
            "cart_speed_m_s": peak_cart_speed,
            "dx_m_s": [peak_dx, peak_dy, peak_dz],
            "drot_rad_s": [peak_drx, peak_dry, peak_drz],
            "max_ori_offset_rad": peak_ori_offset,
        }

    # --- multiband / sine path (legacy interface: rot_a=yaw, rot_b=roll) ----
    yaw, roll = rot_a, rot_b
    # (T, 2) tilt (rx, ry): multiband only, zero unless --amp-rx / --amp-ry are on.
    tilt = getattr(args, "_multiband_tilt", None)
    if tilt is None:
        tilt = np.zeros((yaw.shape[0], 2), dtype=np.float64)
    peak_dyaw = float(np.max(np.abs(np.gradient(yaw, dt_grid)))) if yaw.size > 1 else 0.0
    peak_droll = float(np.max(np.abs(np.gradient(roll, dt_grid)))) if roll.size > 1 else 0.0
    if tilt.shape[0] > 1:
        peak_drx, peak_dry = np.max(np.abs(np.gradient(tilt, dt_grid, axis=0)), axis=0).tolist()
    else:
        peak_drx = peak_dry = 0.0
    # Largest angle between q_des and the anchor, measured on the quaternions
    # (the per-angle amplitudes do not add up: yaw and roll can cancel).
    dot = np.clip(np.abs(quat_des_xyzw @ q_anchor_xyzw), 0.0, 1.0)
    peak_ori_offset = float(np.max(2.0 * np.arccos(dot)))

    print(
        f"[cart_impedance] peak |dx_des| [m/s]: x={peak_dx:.4f}, y={peak_dy:.4f}, "
        f"z={peak_dz:.4f}  (|dx|_max={peak_cart_speed:.4f})"
    )
    print(
        f"[cart_impedance] peak rotation rates [rad/s]: dyaw={peak_dyaw:.4f}, "
        f"droll={peak_droll:.4f}, drx={peak_drx:.4f}, dry={peak_dry:.4f}. "
        f"q_des max ang-offset = {peak_ori_offset:.3f} rad"
    )
    return {
        "cart_speed_m_s": peak_cart_speed,
        "dx_m_s": [peak_dx, peak_dy, peak_dz],
        "dyaw_rad_s": peak_dyaw,
        "droll_rad_s": peak_droll,
        "drx_rad_s": peak_drx,
        "dry_rad_s": peak_dry,
        "max_ori_offset_rad": peak_ori_offset,
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
    path: Path | None,
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
    ring_log: dict | None = None,
) -> dict:
    """Build the sidecar, write it to ``path`` (None = build only) and return it."""
    if args.mode == "multiband":
        # One entry per tone, three per axis.
        tones = multiband_tones(args.profile)
        pos_axes, ori_axes = ("x", "y", "z"), ("yaw", "roll", "rx", "ry")
        pos_freq_set = {axis: [f for f, _, _ in tones[axis]] for axis in pos_axes}
        pos_phase_set = {axis: [ph for _, _, ph in tones[axis]] for axis in pos_axes}
        ori_freq_set = {axis: [f for f, _, _ in tones[axis]] for axis in ori_axes}
        ori_phase_set = {axis: [ph for _, _, ph in tones[axis]] for axis in ori_axes}
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
        amplitude_rad = {"yaw": float(args.amp_yaw), "roll": float(args.amp_roll),
                         "rx": float(args.amp_rx), "ry": float(args.amp_ry)}
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
        "sidecar_path": str(path) if path is not None else None,
        "control_rate_hz": float(args.rate),
        "duration_s": float(duration_s),
        "num_samples": int(num_samples),
        "amp_ramp_s": float(args.amp_ramp) if args.mode == "multiband" else 0.0,
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
            **({"band": args.band, "f0_hz": float(args.f0), "f1_hz": float(args.f1),
                "amp_taper_exp": float(args.amp_taper_exp),
                "ramp_up_s": float(args.ramp_up), "ramp_down_s": float(args.ramp_down)}
                if args.mode == "chirp" else {}),
            # Relative amplitude per tone, same order as freq_set_hz / ori_freq_set_hz.
            **({"profile": args.profile, "ramp_down_s": float(args.amp_ramp_down),
                "tone_rel_amp": {axis: [a for _, a, _ in tones[axis]] for axis in pos_axes + ori_axes}}
                if args.mode == "multiband" else {}),
        },
        "abort": abort,
        "summary": summary,
        # --log-1khz: the ring-log summary (path = the CSV on this machine), None otherwise.
        "ring_log": ring_log,
    }
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"[cart_impedance] wrote sidecar: {path}")
    return payload


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
        # chirp runs 8.0 s; multiband runs 12.0 s (both profiles).
        args.duration = 8.0 if args.mode == "chirp" else (12.0 if args.mode == "multiband" else 4.0)
    # Resolve mode-dependent amplitude defaults.
    # multiband: the --profile's (0.06 on all three).
    # chirp: UR5e-exact magnitudes (0.10/0.10/0.15, Z = 1.5x XY; 0.50/0.25 rad).
    if args.mode == "chirp":
        if args.amp_x is None: args.amp_x = 0.10
        if args.amp_y is None: args.amp_y = 0.10
        if args.amp_z is None: args.amp_z = 0.15
        if args.amp_rx is None: args.amp_rx = 0.50
        if args.amp_ry is None: args.amp_ry = 0.25
        band = CHIRP_BANDS[args.band]
        if args.f0 is None: args.f0 = band["f0"]
        if args.f1 is None: args.f1 = band["f1"]
        if args.amp_taper_exp is None: args.amp_taper_exp = band["amp_taper_exp"]
        if args.ramp_up is None: args.ramp_up = band["ramp_up_s"]
        if args.ramp_down is None: args.ramp_down = band["ramp_down_s"]
        print(f"[cart_impedance] chirp band '{args.band}': {args.f0:g} -> {args.f1:g} Hz, "
              f"amplitude taper exponent {args.amp_taper_exp:g}, ramp {args.ramp_up:g} s up / "
              f"{args.ramp_down:g} s down")
    elif args.mode == "multiband":
        prof = MULTIBAND_PROFILES[args.profile]
        for flag, key in (("amp_x", "amp_x"), ("amp_y", "amp_y"), ("amp_z", "amp_z"),
                          ("amp_yaw", "amp_yaw"), ("amp_roll", "amp_roll"),
                          ("amp_rx", "amp_rx"), ("amp_ry", "amp_ry"),
                          ("amp_ramp", "amp_ramp_s"), ("amp_ramp_down", "ramp_down_s")):
            if getattr(args, flag) is None:
                setattr(args, flag, prof[key])
        print(f"[cart_impedance] multiband profile '{args.profile}': "
              f"{len(multiband_tones(args.profile)['x'])} tones per axis, amp "
              f"{args.amp_x:g}/{args.amp_y:g}/{args.amp_z:g} m, yaw {args.amp_yaw:g} roll {args.amp_roll:g} "
              f"rx {args.amp_rx:g} ry {args.amp_ry:g} rad, fade-in {args.amp_ramp:g} s, "
              f"fade-out {args.amp_ramp_down:g} s")
    if args.log is None and args.log_1khz is None and not args.dry_run:
        print("[cart_impedance] (neither --log-1khz nor --log given, state will not be saved to disk)",
              file=sys.stderr)

    # Both logs are written on this machine, so they must not share a file.
    log_path = Path(args.log).expanduser().resolve() if args.log else None
    ring_path = Path(args.log_1khz).expanduser().resolve() if args.log_1khz else None
    if log_path is not None and ring_path is not None and log_path in (ring_path, targets_path_for(ring_path)):
        print(f"[cart_impedance] --log and --log-1khz would both write {log_path}; give them "
              "different names", file=sys.stderr)
        return 2
    # Sidecar: --sidecar, else next to --log, else next to the 1 kHz CSV --
    # settled after the run, once it is known where that CSV was written.
    sidecar_path: Path | None = None
    if args.sidecar:
        sidecar_path = Path(args.sidecar).expanduser().resolve()
    elif log_path is not None:
        sidecar_path = log_path.with_suffix(".json")

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
        _print_peak_rates(dx_des, quat_des_xyzw, q_anchor_xyzw, yaw, roll, args)
        print(f"[cart_impedance] dry-run: built {n} samples over {args.duration:.2f} s "
              f"@ {args.rate:.1f} Hz; first x_des={x_des[0]}, last={x_des[-1]}")
        return 0

    # ----------------------------------------------------------- live path
    if ring_path is not None:
        # A 1 kHz log that cannot be written should stop the run here, not
        # surface once the arm has moved.
        try:
            ring_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            print(f"[cart_impedance] cannot write --log-1khz {ring_path}: {e}", file=sys.stderr)
            return 2
    cfg = load_config(args.config)
    kp_pos = float(args.kp_pos) if args.kp_pos is not None else float(cfg.control.kp_pos)
    kp_ori = float(args.kp_ori) if args.kp_ori is not None else float(cfg.control.kp_ori)

    if args.mode == "chirp" and args.band == "high" and args.amp_taper_exp < 2.0 and kp_pos > 250.0:
        # The high band's default taper is sized for kp 200/20: a stiffer loop
        # follows the reference further up the band and the force grows with it.
        print(f"[cart_impedance] WARNING: --band high with amp_taper_exp={args.amp_taper_exp:g} is sized "
              f"for kp 200/20, not kp_pos={kp_pos:g} (the model puts the default at ~84 N of peak "
              "EE force at 500/30, against ~33 N at 200/20); consider --amp-taper-exp 2", file=sys.stderr)

    with FrankaTwinClient(cfg) as robot:
        robot.set_gains(
            kp_pos=kp_pos,
            kp_ori=kp_ori,
            error_delta_pos=args.err_delta_pos,
        )
        if args.err_delta_pos is not None:
            print(
                f"[cart_impedance] osc_shm clamp overridden: "
                f"err_delta_pos={args.err_delta_pos}"
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
        peak_rates = _print_peak_rates(dx_des, quat_des_xyzw, q_anchor_xyzw, yaw, roll, args)

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

        # 1 kHz ring log: every controller tick, targets tick-exact. The daemon
        # records in memory on the NUC; the files are written here afterwards.
        # Started right before the first setpoint so the merged CSV begins at
        # the run, stopped right after the loop so it ends with it.
        ring_summary: dict | None = None
        ring_active = False
        if ring_path is not None:
            info = robot.log_start(ring_path)
            ring_active = True
            print(f"[cart_impedance] 1 kHz ring log started (from seq {info['seq_start']}), "
                  f"to be written at {info['path']}")

        def _stop_ring() -> None:
            # Never let a ring-log failure cost the 50 Hz log + sidecar: the
            # error goes to stderr and into the sidecar's "ring_log" instead.
            nonlocal ring_summary, ring_active
            if not ring_active:
                return
            ring_active = False
            try:
                # save=False: the transfer waits until the arm is back at the anchor.
                ring_summary = robot.log_stop(save=False)
            except Exception as e:
                ring_summary = {"error": str(e)}
                print(f"[cart_impedance] ring log stop failed: {e}", file=sys.stderr)
                return
            print(f"[cart_impedance] 1 kHz ring log: {ring_summary['num_frames']} frames "
                  f"(seq {ring_summary['seq_first']}..{ring_summary['seq_last']}, "
                  f"{ring_summary['duration_s']:.3f} s), {ring_summary['num_targets']} targets")
            if ring_summary["dropped_frames"] or ring_summary["resets"]:
                print("[cart_impedance] WARNING: ring log lost frames or saw a controller restart "
                      f"(dropped={ring_summary['dropped_frames']}, gaps={ring_summary['gaps']}, "
                      f"resets={ring_summary['resets']}); not a clean 1 kHz run", file=sys.stderr)

        def _save_ring() -> None:
            # Fetch the rows from the daemon and write the two CSVs here. The
            # daemon keeps the log until the next log_start, so a failure can
            # still be made good with FrankaTwinClient.log_save().
            if ring_summary is None or "error" in ring_summary:
                return
            try:
                try:
                    written = robot.log_save()
                except OSError as e:
                    fallback = Path.cwd() / ring_path.name
                    print(f"[cart_impedance] cannot write {ring_path} ({e}); using {fallback}",
                          file=sys.stderr)
                    written = robot.log_save(fallback)
            except Exception as e:
                ring_summary["save_error"] = str(e)
                print(f"[cart_impedance] could not save the 1 kHz ring log: {e}\n"
                      "[cart_impedance] the daemon keeps it until the next log_start; retry with "
                      "FrankaTwinClient.log_save(path)", file=sys.stderr)
                return
            ring_summary.update(written)
            print(f"[cart_impedance] wrote 1 kHz CSV: {written['path']} "
                  f"(+ {Path(written['targets_path']).name})")

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

            _stop_ring()
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
            _stop_ring()

        _save_ring()
        if log_path is not None:
            _write_csv(
                log_path, t_grid, x_des, dx_des, quat_des_xyzw,
                log_q, log_dq, log_x, log_quat_xyzw, log_period,
                log_tau, log_tau_J,
            )
        def _sidecar(path: Path, csv_path: Path | None) -> Path:
            """Write the sidecar at ``path``, or in the cwd if that fails. Returns where."""
            def write(p: Path) -> Path:
                _write_sidecar(
                    p, args, q_init, x_anchor, q_anchor_xyzw,
                    duration_s=float(args.duration), num_samples=n,
                    csv_path=csv_path, peak_rates=peak_rates,
                    kp_pos=kp_pos, kp_ori=kp_ori,
                    summary=summary, abort=abort,
                    ring_log=ring_summary,
                )
                return p
            try:
                return write(path)
            except OSError as e:
                fallback = Path.cwd() / path.name
                print(f"[cart_impedance] cannot write {path} ({e}); using {fallback}", file=sys.stderr)
                return write(fallback)

        # csv_path names the CSV the sidecar sits next to: the 50 Hz log if
        # there is one, else the 1 kHz log.
        ring_csv = Path(ring_summary["path"]) if ring_summary and ring_summary.get("path") else None
        if sidecar_path is None and ring_path is not None:
            sidecar_path = (ring_csv if ring_csv is not None else ring_path).with_suffix(".json")
        written_sidecar: Path | None = None
        if sidecar_path is not None:
            written_sidecar = _sidecar(sidecar_path, log_path if log_path is not None else ring_csv)
        if ring_csv is not None and written_sidecar != ring_csv.with_suffix(".json"):
            # --log / --sidecar put the sidecar elsewhere: the fit takes the
            # 1 kHz CSV with a sidecar of the same name, so write that pair too.
            _sidecar(ring_csv.with_suffix(".json"), ring_csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
