#!/usr/bin/env python3
"""Sim2real comparison for the single-axis fixed-delta-action test.

Overlays the REAL trajectory (panda_control/examples/fixed_delta_pose_test.py)
and the SIM trajectory (IsaacLab scripts/environments/franka_debug_v1_test.py,
run with --task Franka-Debug-v2) for a CONSTANT per-tick delta action -- e.g.
a pure z push ``--delta 0 0 -0.02``.  Both sides must use matched gains
(default the scripts agree on kp_pos=200/kp_ori=20; pass --kp-pos 500 --kp-ori
30 on real and --kp_pos 500 --kp_rot 30 on sim for the kp500 set).

This is the single-axis analogue of ``compare_sixdof_sim_real.py``.  It uses the
SAME column mapping for the two CSV schemas but drops the ``phase_name``
dependency (the fixed-delta test is one continuous phase):

    quantity         real column(s)              sim column(s)
    -------------    -------------------------   ------------------
    time             t_s                         t_s
    EE displacement  disp_x, disp_y, disp_z      dx, dy, dz
    |rot from start| rot_dev_rad                 rot_dev_rad

We compare DISPLACEMENT (Δ from each side's own start) and ROT_DEV (geodesic
angle from each side's own start quat), both of which are independent of the
absolute base-frame origin -- so the comparison is unaffected by the real arm
and the sim reporting slightly different absolute EE points.

The headline dynamics number for this test is the STEADY-STATE SPEED along the
commanded axis: with a constant delta the controller settles at roughly
``v_ss ≈ (kp_pos/kd_pos)*|delta|``.  We estimate it on each side as the slope of
the commanded-axis displacement over the last ``--tail-frac`` of the run.

Sampling note: the sim script logs only ~21 samples (sample_every = n_steps//20)
while the real loop logs at its re-anchor rate (~50 Hz).  The sim trace is
linearly interpolated onto the real time grid before diffing; the slope fit is
robust to the coarse sim sampling because the tail is near-linear.

Caveat (same as the 6-DOF tool): the sim re-anchors target=current+delta at the
physics rate (~1 kHz) while the real loop re-anchors at ``--rate`` (default 50
Hz).  A lower real re-anchor rate yields a smaller mean tracking error and thus
a SLOWER real steady-state speed -- a known sim/real confound, NOT a dynamics
gap.  Raise the real ``--rate`` to close it.

Usage::

    python scripts/compare_fixed_delta_sim_real.py \\
        --real-csv data/real_delta_z_20260602_120000.csv \\
        --sim-csv  /home/tao/Projects/IsaacLab/logs/franka_debug_v2/videos/trajectory_z_d002_kp500.csv
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

DISP_REAL = ["disp_x", "disp_y", "disp_z"]
DISP_SIM = ["dx", "dy", "dz"]
AXIS_NAMES = ("x", "y", "z")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Single-axis fixed-delta sim2real comparison.")
    p.add_argument("--real-csv", required=True,
                   help="Real fixed_delta CSV (examples/fixed_delta_pose_test.py --log).")
    p.add_argument("--sim-csv", required=True,
                   help="Sim trajectory CSV (franka_debug_v1_test.py, --task Franka-Debug-v2).")
    p.add_argument("--axis", choices=AXIS_NAMES, default=None,
                   help="Commanded axis for the steady-state-speed report. "
                        "Default: auto-detect from the largest |final displacement|.")
    p.add_argument("--tail-frac", type=float, default=0.25,
                   help="Fraction of the run (from the end) used to fit the "
                        "steady-state speed slope (default: 0.25).")
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


def _slope(t: np.ndarray, y: np.ndarray, tail_frac: float) -> float:
    """Least-squares slope [units of y / s] over the last ``tail_frac`` of t."""
    if len(t) < 2:
        return float("nan")
    t0 = t[-1] - tail_frac * (t[-1] - t[0])
    m = t >= t0
    if m.sum() < 2:
        m = np.ones(len(t), dtype=bool)
    coef = np.polyfit(t[m], y[m], 1)
    return float(coef[0])


def main() -> int:
    args = parse_args()
    real_csv = Path(args.real_csv).expanduser().resolve()
    sim_csv = Path(args.sim_csv).expanduser().resolve()
    for pth in (real_csv, sim_csv):
        if not pth.is_file():
            raise FileNotFoundError(pth)

    real = pd.read_csv(real_csv)
    sim = pd.read_csv(sim_csv)

    for col in [*DISP_REAL, "rot_dev_rad", "t_s"]:
        if col not in real.columns:
            raise ValueError(f"real CSV missing column {col!r}")
    for col in [*DISP_SIM, "rot_dev_rad", "t_s"]:
        if col not in sim.columns:
            raise ValueError(f"sim CSV missing column {col!r}")

    t_real = real["t_s"].to_numpy()
    t_sim = sim["t_s"].to_numpy()

    disp_real = real[DISP_REAL].to_numpy()
    rot_real = real["rot_dev_rad"].to_numpy()

    disp_sim = _interp_cols(t_real, t_sim, sim[DISP_SIM].to_numpy())
    rot_sim = _interp_cols(t_real, t_sim, sim[["rot_dev_rad"]].to_numpy())[:, 0]

    d_disp = disp_sim - disp_real           # (N,3) sim - real
    d_rot = rot_sim - rot_real              # (N,)

    # Commanded axis: explicit, else the axis with the largest real final |Δ|.
    final_real = disp_real[-1]
    axis_idx = (AXIS_NAMES.index(args.axis) if args.axis is not None
                else int(np.argmax(np.abs(final_real))))
    axis = AXIS_NAMES[axis_idx]

    # Steady-state speed = slope of commanded-axis displacement over the tail.
    v_real = _slope(t_real, disp_real[:, axis_idx], args.tail_frac)
    # Fit the sim slope on the sim's own (denser-in-time, coarser-in-count) grid.
    v_sim = _slope(t_sim, sim[DISP_SIM[axis_idx]].to_numpy(), args.tail_frac)

    # ---- summary ----------------------------------------------------------
    final_sim = disp_sim[-1]
    print("=" * 88)
    print(f"real: {real_csv.name}  ({len(real)} rows, {t_real[-1]:.2f}s)")
    print(f"sim : {sim_csv.name}  ({len(sim)} rows, {t_sim[-1]:.2f}s -> interp to real grid)")
    print(f"commanded axis: {axis}   (steady-state slope over last {args.tail_frac*100:.0f}% of run)")
    print("=" * 88)
    print(f"{'metric':<26}{'real':>14}{'sim':>14}{'sim - real':>16}")
    print("-" * 88)
    for i, a in enumerate(AXIS_NAMES):
        print(f"final Δ{a} [mm]{'':<13}{final_real[i]*1000:>14.3f}"
              f"{final_sim[i]*1000:>14.3f}{(final_sim[i]-final_real[i])*1000:>16.3f}")
    print("-" * 88)
    for i, a in enumerate(AXIS_NAMES):
        print(f"disp_rms_Δ{a} diff [mm]{'':<6}{'':>14}{'':>14}{_rms(d_disp[:, i])*1000:>16.3f}")
    print(f"|Δpos| rms [mm]{'':<12}{'':>14}{'':>14}{_rms(np.linalg.norm(d_disp, axis=1))*1000:>16.3f}")
    print(f"|Δpos| max [mm]{'':<12}{'':>14}{'':>14}"
          f"{float(np.max(np.linalg.norm(d_disp, axis=1)))*1000:>16.3f}")
    print("-" * 88)
    print(f"v_ss[{axis}] [mm/s]{'':<11}{v_real*1000:>14.3f}{v_sim*1000:>14.3f}"
          f"{(v_sim-v_real)*1000:>16.3f}")
    if abs(v_real) > 1e-9:
        print(f"v_ss[{axis}] sim/real ratio{'':<3}{'':>14}{'':>14}{v_sim/v_real:>16.3f}")
    print("-" * 88)
    print(f"rot_dev rms [deg]{'':<10}{_rms(rot_real)*180/np.pi:>14.3f}"
          f"{_rms(rot_sim)*180/np.pi:>14.3f}{'':>16}")
    print(f"rot_dev max [deg]{'':<10}{float(np.max(np.abs(rot_real)))*180/np.pi:>14.3f}"
          f"{float(np.max(np.abs(rot_sim)))*180/np.pi:>14.3f}{'':>16}")
    print("=" * 88)
    print("notes: Δpos = |sim_disp - real_disp| (frame-origin independent);")
    print("       v_ss = slope of commanded-axis displacement over the tail (the dynamics number).")
    print("       a sim/real v_ss ratio > 1 is partly the 1kHz-vs-50Hz re-anchor confound -- raise real --rate.")
    print("=" * 88)

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

    # 1) displacement per axis
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, lab, idx in zip(axes, AXIS_NAMES, range(3)):
        ax.plot(t_real, disp_real[:, idx] * 1000, "-", color="C0", lw=1.3, label="real")
        ax.plot(t_real, disp_sim[:, idx] * 1000, "-", color="C1", lw=1.3, alpha=0.85, label="sim")
        ax.set_ylabel(f"Δ{lab} [mm]")
        ax.grid(True, alpha=0.3)
        if idx == 0:
            ax.legend(loc="upper left")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"EE displacement from start: real vs sim (commanded axis: {axis})")
    fig.tight_layout()
    fig.savefig(out_dir / "displacement_timeseries.png", dpi=args.dpi)
    plt.close(fig)

    # 2) displacement error magnitude
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t_real, np.linalg.norm(d_disp, axis=1) * 1000, "-", color="C3", lw=1.2)
    ax.set_xlabel("t [s]"); ax.set_ylabel("|sim - real| [mm]")
    ax.set_title("EE displacement error magnitude (sim vs real)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_dir / "displacement_error.png", dpi=args.dpi); plt.close(fig)

    # 3) commanded-axis displacement with steady-state slope fits
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t_real, disp_real[:, axis_idx] * 1000, "-", color="C0", lw=1.4,
            label=f"real (v_ss={v_real*1000:.1f} mm/s)")
    ax.plot(t_real, disp_sim[:, axis_idx] * 1000, "-", color="C1", lw=1.4, alpha=0.85,
            label=f"sim (v_ss={v_sim*1000:.1f} mm/s)")
    ax.set_xlabel("t [s]"); ax.set_ylabel(f"Δ{axis} [mm]")
    ax.set_title(f"Commanded-axis ({axis}) displacement: real vs sim")
    ax.grid(True, alpha=0.3); ax.legend(loc="best")
    fig.tight_layout(); fig.savefig(out_dir / "commanded_axis_displacement.png", dpi=args.dpi)
    plt.close(fig)

    # 4) rot deviation overlay
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t_real, np.degrees(rot_real), "-", color="C0", lw=1.3, label="real")
    ax.plot(t_real, np.degrees(rot_sim), "-", color="C1", lw=1.3, alpha=0.85, label="sim")
    ax.set_xlabel("t [s]"); ax.set_ylabel("|rotation from start| [deg]")
    ax.set_title("EE orientation deviation: real vs sim")
    ax.grid(True, alpha=0.3); ax.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(out_dir / "rotdev_timeseries.png", dpi=args.dpi); plt.close(fig)

    print(f"[compare] figures saved under {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
