#!/usr/bin/env python3
"""Plot actual vs target EE trajectory per dimension from a cart_impedance log.

Seven panels: x/y/z position (actual `x_*` vs target `x_des_*`) and the four
quaternion components qx/qy/qz/qw (actual `quat_*` vs target `quat_des_*`,
xyzw). Actual quats are hemisphere-aligned to the target (q and -q are the
same rotation) so sign flips don't show up as jumps.

Usage (needs pandas + matplotlib -> run with the `isaac`/`isaaclab` env):
  python scripts/plot_ee_tracking.py data/real_v4chirp_load068_<ts>.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd


def _align_hemisphere(q: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
    """Flip the sign of each quat so it lies in the same hemisphere as q_ref."""
    sign = np.where(np.einsum("ij,ij->i", q, q_ref) < 0.0, -1.0, 1.0)
    return q * sign[:, None]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("csv", type=str, help="CSV log from examples/cart_impedance.py")
    p.add_argument("--out", type=str, default=None,
                   help="Output figure path (default: <csv>_ee_tracking.png)")
    args = p.parse_args()

    csv_path = Path(args.csv).expanduser().resolve()
    df = pd.read_csv(csv_path)
    t = df["t_s"].to_numpy()

    x_act = df[["x_x", "x_y", "x_z"]].to_numpy()
    x_des = df[["x_des_x", "x_des_y", "x_des_z"]].to_numpy()
    q_act = df[["quat_x", "quat_y", "quat_z", "quat_w"]].to_numpy()
    q_des = df[["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]].to_numpy()

    # q and -q encode the same rotation; align actual to the target hemisphere
    # so the component plots don't show spurious sign-flip jumps.
    q_act = _align_hemisphere(q_act, q_des)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
    pos_labels = ["x", "y", "z"]
    for i in range(3):
        ax = axes[i]
        ax.plot(t, 1000.0 * x_des[:, i], "k--", lw=1.0, label="target")
        ax.plot(t, 1000.0 * x_act[:, i], lw=1.0, label="actual")
        err = x_act[:, i] - x_des[:, i]
        ax.set_ylabel(f"{pos_labels[i]} [mm]")
        ax.set_title(
            f"pos {pos_labels[i]}: rms err = {1000.0 * np.sqrt(np.mean(err**2)):.1f} mm, "
            f"max |err| = {1000.0 * np.max(np.abs(err)):.1f} mm", fontsize=9)
        if i == 0:
            ax.legend(fontsize=8, loc="upper right")
    quat_labels = ["qx", "qy", "qz", "qw"]
    for i in range(4):
        ax = axes[3 + i]
        ax.plot(t, q_des[:, i], "k--", lw=1.0, label="target")
        ax.plot(t, q_act[:, i], lw=1.0, label="actual")
        err = q_act[:, i] - q_des[:, i]
        ax.set_ylabel(quat_labels[i])
        ax.set_title(
            f"{quat_labels[i]}: rms err = {np.sqrt(np.mean(err**2)):.4f}, "
            f"max |err| = {np.max(np.abs(err)):.4f}", fontsize=9)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"{csv_path.name} — EE tracking (actual vs target)")
    fig.tight_layout()

    out = Path(args.out).expanduser().resolve() if args.out \
        else csv_path.with_name(csv_path.stem + "_ee_tracking.png")
    fig.savefig(out, dpi=150)
    print(f"wrote figure: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
