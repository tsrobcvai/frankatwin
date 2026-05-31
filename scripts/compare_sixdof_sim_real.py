#!/usr/bin/env python3
"""Per-phase / per-tick sim2real comparison for the 7-phase 6-DOF test.

Overlays the REAL trajectory (panda_control/examples/six_dof_pose_test.py) and
the SIM trajectory (IsaacLab scripts/environments/franka_debug_v2_six_dof_test.py)
that were produced with matched gains (kp_pos=500, kp_ori=30, kd=2*sqrt(kp)),
matched schedule (7 phases, no settle, 14 s) and matched torque slew (800 Nm/s).

The two CSVs use different column conventions, so this tool maps them:

    quantity         real column(s)              sim column(s)
    -------------    -------------------------   ------------------
    time             t_s                         t_s
    EE displacement  disp_x, disp_y, disp_z      dx, dy, dz
    |rot from start| rot_dev_rad                 rot_dev_rad
    phase label      phase_name                  phase_name

We compare DISPLACEMENT (Δ from each side's own start) and ROT_DEV (geodesic
angle from each side's own start quat), both of which are independent of the
absolute base-frame origin -- so the comparison is unaffected by the real
arm and the sim reporting slightly different absolute EE points.

CAVEAT (printed in the summary): in the pure-rotation phases (phase4_rx..
phase6_rz) the *position* displacement of a body-fixed point depends on which
point is tracked (real libfranka kEndEffector vs sim panda_fingertip_centered).
If those two reference points differ, the rotation-phase position delta carries
a reference-point component that is NOT a dynamics mismatch.  ROT_DEV and the
translation phases (phase1_x..phase3_z) are clean for all reference points.

The sim logs at the physics rate (~1 kHz) and the real at the re-anchor rate
(~50 Hz); sim is linearly interpolated onto the real time grid before diffing.

Usage::

    python scripts/compare_sixdof_sim_real.py \\
        --real-csv data/real_sixdof_20260531_114913.csv \\
        --sim-csv  /home/tao/Projects/IsaacLab/logs/franka_debug_v2/videos/trajectory_v2_nosettle_kp500.csv
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

DISP_REAL = ["disp_x", "disp_y", "disp_z"]
DISP_SIM = ["dx", "dy", "dz"]
TRANSLATION_PHASES = {"phase1_x", "phase2_y", "phase3_z"}
ROTATION_PHASES = {"phase4_rx", "phase5_ry", "phase6_rz"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="6-DOF sim2real trajectory comparison.")
    p.add_argument("--real-csv", required=True, help="Real six_dof CSV.")
    p.add_argument("--sim-csv", required=True, help="Sim franka_debug_v2 trajectory CSV.")
    p.add_argument("--out-dir", default=None,
                   help="PNG output dir. Default: <real-csv-dir>/compare_<real-stem>.")
    p.add_argument("--show", action="store_true", help="Show figures interactively.")
    p.add_argument("--dpi", type=int, default=140)
    return p.parse_args()


def _interp_cols(t_dst: np.ndarray, t_src: np.ndarray, arr: np.ndarray) -> np.ndarray:
    out = np.empty((len(t_dst), arr.shape[1]), dtype=np.float64)
    for c in range(arr.shape[1]):
        out[:, c] = np.interp(t_dst, t_src, arr[:, c])
    return out


def _rms(a: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(a))))


def main() -> int:
    args = parse_args()
    real_csv = Path(args.real_csv).expanduser().resolve()
    sim_csv = Path(args.sim_csv).expanduser().resolve()
    for pth in (real_csv, sim_csv):
        if not pth.is_file():
            raise FileNotFoundError(pth)

    real = pd.read_csv(real_csv)
    sim = pd.read_csv(sim_csv)

    for col in [*DISP_REAL, "rot_dev_rad", "t_s", "phase_name"]:
        if col not in real.columns:
            raise ValueError(f"real CSV missing column {col!r}")
    for col in [*DISP_SIM, "rot_dev_rad", "t_s", "phase_name"]:
        if col not in sim.columns:
            raise ValueError(f"sim CSV missing column {col!r}")

    t_real = real["t_s"].to_numpy()
    t_sim = sim["t_s"].to_numpy()

    disp_real = real[DISP_REAL].to_numpy()
    rot_real = real["rot_dev_rad"].to_numpy()
    phase_real = real["phase_name"].astype(str).to_numpy()

    disp_sim = _interp_cols(t_real, t_sim, sim[DISP_SIM].to_numpy())
    rot_sim = _interp_cols(t_real, t_sim, sim[["rot_dev_rad"]].to_numpy())[:, 0]

    d_disp = disp_sim - disp_real           # (N,3) sim - real
    d_rot = rot_sim - rot_real              # (N,)

    # ---- per-phase + overall summary --------------------------------------
    print("=" * 92)
    print(f"real: {real_csv.name}  ({len(real)} rows, {t_real[-1]:.2f}s)")
    print(f"sim : {sim_csv.name}  ({len(sim)} rows, {t_sim[-1]:.2f}s -> interp to real grid)")
    print("=" * 92)
    hdr = (f"{'phase':<13}{'n':>4} | "
           f"{'|Δpos|rms':>10}{'|Δpos|max':>10} (mm) | "
           f"{'Δrot rms':>9}{'Δrot max':>9} (deg) | kind")
    print(hdr)
    print("-" * 92)

    def _row(label, mask):
        if mask.sum() == 0:
            return
        dd = d_disp[mask]
        dr = d_rot[mask]
        pos_rms = _rms(np.linalg.norm(dd, axis=1)) * 1000.0
        pos_max = float(np.max(np.linalg.norm(dd, axis=1))) * 1000.0
        rot_rms = _rms(dr) * 180.0 / np.pi
        rot_max = float(np.max(np.abs(dr))) * 180.0 / np.pi
        ph = label
        kind = ("translation (clean)" if ph in TRANSLATION_PHASES
                else "rotation (pos has ref-pt confound)" if ph in ROTATION_PHASES
                else "combined" if ph.startswith("phase7")
                else "")
        print(f"{label:<13}{int(mask.sum()):>4} | "
              f"{pos_rms:>10.3f}{pos_max:>10.3f}      | "
              f"{rot_rms:>9.3f}{rot_max:>9.3f}      | {kind}")

    for ph in ["phase1_x", "phase2_y", "phase3_z", "phase4_rx",
               "phase5_ry", "phase6_rz", "phase7_all6"]:
        _row(ph, phase_real == ph)
    print("-" * 92)
    _row("ALL", np.ones(len(t_real), dtype=bool))
    # translation-only aggregate (cleanest position metric)
    trans_mask = np.isin(phase_real, list(TRANSLATION_PHASES))
    _row("translation", trans_mask)
    print("=" * 92)
    print("notes: Δpos = |sim_disp - real_disp| (frame-origin independent);")
    print("       Δrot = sim_rot_dev - real_rot_dev (geodesic angle from each start, ref-pt independent).")
    print("       rotation-phase Δpos includes a real-vs-sim EE reference-point component, not just dynamics.")
    print("=" * 92)

    # ---- figures ----------------------------------------------------------
    if args.out_dir is None:
        out_dir = real_csv.parent / f"compare_{real_csv.stem}"
    else:
        out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not args.show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # phase boundary times (start of each phase) for vertical markers
    bounds = []
    last = None
    for i, ph in enumerate(phase_real):
        if ph != last and ph not in ("reset", ""):
            bounds.append((t_real[i], ph))
            last = ph

    def _mark(ax):
        for tb, ph in bounds:
            ax.axvline(tb, color="k", alpha=0.12, linewidth=0.8)

    # 1) displacement per axis
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, lab, idx in zip(axes, ("x", "y", "z"), range(3)):
        ax.plot(t_real, disp_real[:, idx] * 1000, "-", color="C0", lw=1.3, label="real")
        ax.plot(t_real, disp_sim[:, idx] * 1000, "-", color="C1", lw=1.3, alpha=0.85, label="sim")
        ax.set_ylabel(f"Δ{lab} [mm]")
        ax.grid(True, alpha=0.3)
        _mark(ax)
        if idx == 0:
            ax.legend(loc="upper left")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("EE displacement from start: real vs sim (per axis)")
    fig.tight_layout()
    fig.savefig(out_dir / "displacement_timeseries.png", dpi=args.dpi)
    plt.close(fig)

    # 2) displacement error magnitude
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t_real, np.linalg.norm(d_disp, axis=1) * 1000, "-", color="C3", lw=1.2)
    _mark(ax)
    ax.set_xlabel("t [s]"); ax.set_ylabel("|sim - real| [mm]")
    ax.set_title("EE displacement error magnitude (sim vs real)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_dir / "displacement_error.png", dpi=args.dpi); plt.close(fig)

    # 3) rot deviation overlay
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t_real, np.degrees(rot_real), "-", color="C0", lw=1.3, label="real")
    ax.plot(t_real, np.degrees(rot_sim), "-", color="C1", lw=1.3, alpha=0.85, label="sim")
    _mark(ax)
    ax.set_xlabel("t [s]"); ax.set_ylabel("|rotation from start| [deg]")
    ax.set_title("EE orientation deviation: real vs sim")
    ax.grid(True, alpha=0.3); ax.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(out_dir / "rotdev_timeseries.png", dpi=args.dpi); plt.close(fig)

    # 4) 3D displacement trajectory
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(disp_real[:, 0] * 1000, disp_real[:, 1] * 1000, disp_real[:, 2] * 1000,
            "-", color="C0", lw=1.2, label="real")
    ax.plot(disp_sim[:, 0] * 1000, disp_sim[:, 1] * 1000, disp_sim[:, 2] * 1000,
            "-", color="C1", lw=1.2, alpha=0.85, label="sim")
    ax.scatter([0], [0], [0], color="k", s=20, label="start")
    ax.set_xlabel("Δx [mm]"); ax.set_ylabel("Δy [mm]"); ax.set_zlabel("Δz [mm]")
    ax.set_title("EE displacement trajectory (3D)"); ax.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(out_dir / "traj3d_displacement.png", dpi=args.dpi); plt.close(fig)

    print(f"[compare] figures saved under {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
