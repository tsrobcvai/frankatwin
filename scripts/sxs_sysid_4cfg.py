#!/usr/bin/env python3
"""4-config side-by-side comparison: baseline / v1 / v2 / v3 on one trajectory.

Generates one composite figure each for:
  * EE position timeseries (4 columns × 3 rows: x, y, z)
  * Joint position timeseries (7 rows × 4 columns: per-joint q)
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def interp(t_real: np.ndarray, t_sim: np.ndarray, arr: np.ndarray) -> np.ndarray:
    if len(t_real) == len(t_sim) and np.allclose(t_real, t_sim, atol=1e-4):
        return arr
    return np.interp(t_real, t_sim, arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real-csv", required=True)
    ap.add_argument("--sim-baseline-csv", required=True)
    ap.add_argument("--sim-v1-csv", required=True)
    ap.add_argument("--sim-v2-csv", required=True)
    ap.add_argument("--sim-v3-csv", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--title-suffix", default="")
    args = ap.parse_args()

    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    real = pd.read_csv(args.real_csv)
    sims = {
        "baseline\n(no sysid)": pd.read_csv(args.sim_baseline_csv),
        "v1 sysid\n(step5b only)": pd.read_csv(args.sim_v1_csv),
        "v2 sysid\n(step5b+5c)": pd.read_csv(args.sim_v2_csv),
        "v3 sysid\n(step5b+5c+5d)": pd.read_csv(args.sim_v3_csv),
    }
    t_real = real["t_s"].to_numpy()

    fig, axes = plt.subplots(3, 4, figsize=(18, 9), sharex=True)
    for col, (lbl, sim) in enumerate(sims.items()):
        t_sim = sim["t_s"].to_numpy()
        for row, axis in enumerate("xyz"):
            ax = axes[row, col]
            ax.plot(t_real, real[f"x_des_{axis}"], "--", color="gray", lw=1.2, label="target")
            ax.plot(t_real, real[f"x_{axis}"], "-", color="C0", lw=1.0, label="real")
            ax.plot(t_real, interp(t_real, t_sim, sim[f"x_{axis}"].to_numpy()), "-", color="C1", lw=1.0, alpha=0.85, label="sim")
            ax.grid(True, alpha=0.3)
            if col == 0:
                ax.set_ylabel(f"{axis} [m]")
            if row == 0:
                ax.set_title(lbl, fontsize=11)
            if row == 0 and col == 0:
                ax.legend(loc="upper right", fontsize=8)
            if row == 2:
                ax.set_xlabel("t [s]")
    suptitle = f"EE position{args.title_suffix}: target vs real vs sim (4 sysid configs)"
    fig.suptitle(suptitle, fontsize=12)
    fig.tight_layout()
    fig.savefig(out_dir / "sxs_position.png", dpi=130)
    plt.close(fig)
    print(f"wrote {out_dir / 'sxs_position.png'}")

    fig, axes = plt.subplots(7, 4, figsize=(18, 16), sharex=True)
    for col, (lbl, sim) in enumerate(sims.items()):
        t_sim = sim["t_s"].to_numpy()
        for j in range(7):
            ax = axes[j, col]
            ax.plot(t_real, real[f"q{j+1}"], "-", color="C0", lw=1.0, label="real")
            ax.plot(t_real, interp(t_real, t_sim, sim[f"q{j+1}"].to_numpy()), "-", color="C1", lw=1.0, alpha=0.85, label="sim")
            ax.grid(True, alpha=0.3)
            if col == 0:
                ax.set_ylabel(f"j{j+1}\nq [rad]", fontsize=9)
            if j == 0:
                ax.set_title(lbl, fontsize=11)
            if j == 0 and col == 0:
                ax.legend(loc="upper right", fontsize=8)
            if j == 6:
                ax.set_xlabel("t [s]")
    fig.suptitle(f"Joint position{args.title_suffix}: real vs sim (4 sysid configs)", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_dir / "sxs_joints.png", dpi=130)
    plt.close(fig)
    print(f"wrote {out_dir / 'sxs_joints.png'}")


if __name__ == "__main__":
    main()
