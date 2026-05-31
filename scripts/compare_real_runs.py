#!/usr/bin/env python3
"""Compare two REAL six_dof_pose_test runs (e.g. with-camera vs without-camera).

Both inputs are CSVs written by panda_control/examples/six_dof_pose_test.py, so
they share the same schema (t_s, phase_name, disp_x/y/z, rot_dev_rad, q0..q6,
dq0..dq6, seq, ...).  This is the real-vs-real counterpart of
compare_sixdof_sim_real.py and additionally diffs the joint trajectories.

Use it to quantify how an end-effector payload (a mounted camera) changes the
arm's response under the identical 6-DOF chase: same controller, same gains,
same schedule -> any trajectory delta is the payload's dynamic effect (added
mass/inertia, and -- if the libfranka load was not reconfigured -- uncompensated
camera weight showing up as extra sag).

Compares (run B - run A):
  * EE displacement from start  (disp_x/y/z)         -> position effect
  * |rotation from start|       (rot_dev_rad)        -> orientation effect
  * per-joint q and dq          (q0..q6, dq0..dq6)   -> where in the arm it shows

IMPORTANT: for a clean comparison both runs must start from the SAME pose --
run examples/reset_home.py before each, so disp/rot are measured from home.

Loading uses only numpy + the stdlib csv module (runs in the `panda` env).
matplotlib is optional: if unavailable the numeric tables still print and plots
are skipped (run with the `isaac`/`isaaclab` env python to get the figures).

Usage::

    python scripts/compare_real_runs.py \\
        --csv-a data/real_sixdof_20260531_114913.csv      --label-a nocam \\
        --csv-b data/real_sixdof_withcam_YYYYMMDD_HHMMSS.csv --label-b withcam
"""

from __future__ import annotations

import argparse
import csv as _csv
import sys
from pathlib import Path

import numpy as np

DISP = ["disp_x", "disp_y", "disp_z"]
QCOLS = [f"q{i}" for i in range(7)]
DQCOLS = [f"dq{i}" for i in range(7)]
PHASES = ["phase1_x", "phase2_y", "phase3_z", "phase4_rx",
          "phase5_ry", "phase6_rz", "phase7_all6"]
TRANSLATION = {"phase1_x", "phase2_y", "phase3_z"}
ROTATION = {"phase4_rx", "phase5_ry", "phase6_rz"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Compare two real six_dof runs (e.g. cam vs no-cam).")
    p.add_argument("--csv-a", required=True, help="Baseline real CSV (e.g. no camera).")
    p.add_argument("--csv-b", required=True, help="Second real CSV (e.g. with camera).")
    p.add_argument("--label-a", default="A", help="Label for csv-a (e.g. nocam).")
    p.add_argument("--label-b", default="B", help="Label for csv-b (e.g. withcam).")
    p.add_argument("--out-dir", default=None,
                   help="PNG dir. Default <csv-b-dir>/compare_<labelB>_vs_<labelA>.")
    p.add_argument("--show", action="store_true")
    p.add_argument("--dpi", type=int, default=140)
    return p.parse_args()


def load(path: Path) -> dict:
    with open(path) as f:
        rows = list(_csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} is empty")
    cols = rows[0].keys()
    need = ["t_s", "phase_name", *DISP, "rot_dev_rad"]
    missing = [c for c in need if c not in cols]
    if missing:
        raise ValueError(f"{path} missing columns: {missing}")
    out = {
        "t": np.array([float(r["t_s"]) for r in rows]),
        "phase": np.array([str(r["phase_name"]) for r in rows]),
        "disp": np.array([[float(r[c]) for c in DISP] for r in rows]),
        "rot": np.array([float(r["rot_dev_rad"]) for r in rows]),
    }
    out["have_q"] = all(c in cols for c in QCOLS)
    out["have_dq"] = all(c in cols for c in DQCOLS)
    if out["have_q"]:
        out["q"] = np.array([[float(r[c]) for c in QCOLS] for r in rows])
    if out["have_dq"]:
        out["dq"] = np.array([[float(r[c]) for c in DQCOLS] for r in rows])
    return out


def interp_to(t_dst: np.ndarray, t_src: np.ndarray, arr: np.ndarray) -> np.ndarray:
    if arr.ndim == 1:
        return np.interp(t_dst, t_src, arr)
    out = np.empty((len(t_dst), arr.shape[1]))
    for c in range(arr.shape[1]):
        out[:, c] = np.interp(t_dst, t_src, arr[:, c])
    return out


def rms(a: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(a))))


def main() -> int:
    args = parse_args()
    csv_a = Path(args.csv_a).expanduser().resolve()
    csv_b = Path(args.csv_b).expanduser().resolve()
    for p in (csv_a, csv_b):
        if not p.is_file():
            raise FileNotFoundError(p)
    A = load(csv_a)
    B = load(csv_b)
    la, lb = args.label_a, args.label_b

    t = A["t"]                       # compare on A's time grid
    disp_a = A["disp"]
    rot_a = A["rot"]
    phase_a = A["phase"]
    disp_b = interp_to(t, B["t"], B["disp"])
    rot_b = interp_to(t, B["t"], B["rot"])

    d_disp = disp_b - disp_a         # (N,3): B - A
    d_rot = rot_b - rot_a            # (N,)

    have_q = A["have_q"] and B["have_q"]
    if have_q:
        q_a = A["q"]
        q_b = interp_to(t, B["t"], B["q"])
        d_q = q_b - q_a              # (N,7)

    print("=" * 96)
    print(f"A ({la}): {csv_a.name}  ({len(A['t'])} rows, {A['t'][-1]:.2f}s)")
    print(f"B ({lb}): {csv_b.name}  ({len(B['t'])} rows, {B['t'][-1]:.2f}s -> interp to A grid)")
    print(f"final |disp| {la}={np.linalg.norm(disp_a[-1])*1000:.1f}mm  "
          f"{lb}={np.linalg.norm(disp_b[-1])*1000:.1f}mm   "
          f"final rot {la}={np.degrees(rot_a[-1]):.2f}deg {lb}={np.degrees(rot_b[-1]):.2f}deg")
    print("=" * 96)
    print(f"{'phase':<13}{'n':>4} | {'|Δpos|rms':>10}{'|Δpos|max':>10} (mm) | "
          f"{'Δrot rms':>9}{'Δrot max':>9} (deg) | "
          f"{'maxjoint|Δq|':>12} (mrad)")
    print("-" * 96)

    def row(label, mask):
        if mask.sum() == 0:
            return
        dd = d_disp[mask]
        dr = d_rot[mask]
        pos_rms = rms(np.linalg.norm(dd, axis=1)) * 1000
        pos_max = float(np.max(np.linalg.norm(dd, axis=1))) * 1000
        rot_rms = rms(dr) * 180 / np.pi
        rot_max = float(np.max(np.abs(dr))) * 180 / np.pi
        qstr = ""
        if have_q:
            dqj = np.abs(d_q[mask])
            jmax = int(np.argmax(dqj.max(axis=0)))
            qstr = f"j{jmax}:{dqj.max()*1000:.2f}"
        print(f"{label:<13}{int(mask.sum()):>4} | {pos_rms:>10.3f}{pos_max:>10.3f}      | "
              f"{rot_rms:>9.3f}{rot_max:>9.3f}      | {qstr:>12}")

    for ph in PHASES:
        row(ph, phase_a == ph)
    print("-" * 96)
    row("ALL", np.isin(phase_a, PHASES))
    row("translation", np.isin(phase_a, list(TRANSLATION)))
    print("=" * 96)
    print(f"Δ = B({lb}) - A({la}).  Both start from home, so Δpos == absolute EE pose difference.")
    print("=" * 96)

    # ---- figures (optional) ----------------------------------------------
    try:
        if not args.show:
            import matplotlib
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[compare] matplotlib unavailable ({e}); skipping plots. "
              f"Run with the isaac/isaaclab env python for figures.")
        return 0

    if args.out_dir is None:
        out_dir = csv_b.parent / f"compare_{lb}_vs_{la}"
    else:
        out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    bounds = []
    last = None
    for i, ph in enumerate(phase_a):
        if ph != last and ph in PHASES:
            bounds.append(t[i]); last = ph

    def mark(ax):
        for tb in bounds:
            ax.axvline(tb, color="k", alpha=0.12, lw=0.8)

    # displacement per axis
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, lab, idx in zip(axes, ("x", "y", "z"), range(3)):
        ax.plot(t, disp_a[:, idx] * 1000, color="C0", lw=1.3, label=la)
        ax.plot(t, disp_b[:, idx] * 1000, color="C1", lw=1.3, alpha=0.85, label=lb)
        ax.set_ylabel(f"Δ{lab} [mm]"); ax.grid(True, alpha=0.3); mark(ax)
        if idx == 0:
            ax.legend(loc="upper left")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"EE displacement from start: {la} vs {lb}")
    fig.tight_layout(); fig.savefig(out_dir / "displacement_timeseries.png", dpi=args.dpi); plt.close(fig)

    # displacement error magnitude
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, np.linalg.norm(d_disp, axis=1) * 1000, color="C3", lw=1.2)
    mark(ax); ax.set_xlabel("t [s]"); ax.set_ylabel(f"|{lb} - {la}| [mm]")
    ax.set_title("EE displacement difference (with-cam effect)"); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_dir / "displacement_diff.png", dpi=args.dpi); plt.close(fig)

    # rot_dev overlay
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, np.degrees(rot_a), color="C0", lw=1.3, label=la)
    ax.plot(t, np.degrees(rot_b), color="C1", lw=1.3, alpha=0.85, label=lb)
    mark(ax); ax.set_xlabel("t [s]"); ax.set_ylabel("|rotation from start| [deg]")
    ax.set_title(f"EE orientation deviation: {la} vs {lb}"); ax.grid(True, alpha=0.3); ax.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(out_dir / "rotdev_timeseries.png", dpi=args.dpi); plt.close(fig)

    # joints
    if have_q:
        fig, axes = plt.subplots(7, 1, figsize=(11, 14), sharex=True)
        for j in range(7):
            axes[j].plot(t, q_a[:, j], color="C0", lw=1.0, label=la if j == 0 else None)
            axes[j].plot(t, q_b[:, j], color="C1", lw=1.0, alpha=0.85, label=lb if j == 0 else None)
            axes[j].set_ylabel(f"q{j} [rad]"); axes[j].grid(True, alpha=0.3); mark(axes[j])
            if j == 0:
                axes[j].legend(loc="upper left")
        axes[-1].set_xlabel("t [s]")
        fig.suptitle(f"Joint positions: {la} vs {lb}")
        fig.tight_layout(); fig.savefig(out_dir / "joints_timeseries.png", dpi=args.dpi); plt.close(fig)

    # 3D displacement
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(disp_a[:, 0]*1000, disp_a[:, 1]*1000, disp_a[:, 2]*1000, color="C0", lw=1.2, label=la)
    ax.plot(disp_b[:, 0]*1000, disp_b[:, 1]*1000, disp_b[:, 2]*1000, color="C1", lw=1.2, alpha=0.85, label=lb)
    ax.scatter([0], [0], [0], color="k", s=20, label="start")
    ax.set_xlabel("Δx [mm]"); ax.set_ylabel("Δy [mm]"); ax.set_zlabel("Δz [mm]")
    ax.set_title("EE displacement trajectory (3D)"); ax.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(out_dir / "traj3d_displacement.png", dpi=args.dpi); plt.close(fig)

    print(f"[compare] figures saved under {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
