#!/usr/bin/env python3
"""Check a cart_impedance.py run log against the Panda joint torque limits.

Reads the CSV written by examples/cart_impedance.py (shm v3+, i.e. with the
tau1-7 / tau_J1-7 columns) and reports, per joint:

  * max |tau_J|   -- measured link-side torque, gravity INCLUDED. This is the
                     number to compare against the actuator limits
                     (87/87/87/87/12/12/12 Nm).
  * max |tau|     -- commanded impedance torque (gravity excluded), useful to
                     see how much of the budget the controller itself uses.
  * limit fraction and verdict.

Also writes a 7-panel time-series figure (tau_J per joint with +/- limit
lines) next to the CSV.

Usage (needs pandas + matplotlib -> run with the `isaac`/`isaaclab` env):
  python scripts/check_torque_limits.py data/chirp_low_fit.csv
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

TAU_LIMITS = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
# Franka's "sustained" torque spec (continuous operation) is lower than the
# peak limit; flag anything above this fraction of the peak limit as worth a
# closer look even if it never trips the hard limit.
WARN_FRAC = 0.8


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("csv", type=str, help="CSV log from examples/cart_impedance.py")
    p.add_argument("--out", type=str, default=None,
                   help="Output figure path (default: <csv>_tau.png)")
    args = p.parse_args()

    csv_path = Path(args.csv).expanduser().resolve()
    df = pd.read_csv(csv_path)

    tau_j_cols = [f"tau_J{i}" for i in range(1, 8)]
    tau_cols = [f"tau{i}" for i in range(1, 8)]
    if not all(c in df.columns for c in tau_j_cols):
        print(f"ERROR: {csv_path.name} has no tau_J1-7 columns -- "
              "was it recorded with the shm v3 stack?", file=sys.stderr)
        return 1

    t = df["t_s"].to_numpy()
    tau_j = df[tau_j_cols].to_numpy()
    tau_cmd = df[tau_cols].to_numpy() if all(c in df.columns for c in tau_cols) else None

    if not np.isfinite(tau_j).any():
        print("ERROR: tau_J is all-NaN (daemon predates shm v3?)", file=sys.stderr)
        return 1

    abs_max = np.nanmax(np.abs(tau_j), axis=0)
    rms = np.sqrt(np.nanmean(tau_j ** 2, axis=0))
    frac = abs_max / TAU_LIMITS

    print(f"\n{csv_path.name}: {len(t)} samples over {t[-1] - t[0]:.2f} s\n")
    print(f"{'joint':>6} {'limit':>7} {'max|tau_J|':>11} {'rms':>8} "
          f"{'max|tau_cmd|':>13} {'% limit':>8}  verdict")
    worst = -1.0
    for j in range(7):
        cmd_str = f"{np.nanmax(np.abs(tau_cmd[:, j])):13.2f}" if tau_cmd is not None else " " * 13
        verdict = ("EXCEEDED" if frac[j] >= 1.0
                   else "WARN" if frac[j] > WARN_FRAC else "ok")
        print(f"{f'j{j+1}':>6} {TAU_LIMITS[j]:7.1f} {abs_max[j]:11.2f} "
              f"{rms[j]:8.2f} {cmd_str} {100.0 * frac[j]:7.1f}%  {verdict}")
        worst = max(worst, frac[j])
    print(f"\npeak usage: {100.0 * worst:.1f}% of limit "
          f"(j{int(np.argmax(frac)) + 1})")
    if worst >= 1.0:
        print("VERDICT: torque limit EXCEEDED -- payload/trajectory too aggressive.")
    elif worst > WARN_FRAC:
        print(f"VERDICT: within limits but above {100 * WARN_FRAC:.0f}% -- "
              "reduce amplitude/frequency or payload margin is thin.")
    else:
        print("VERDICT: comfortably within joint torque limits.")

    # ------------------------------------------------------------- figure
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(7, 1, figsize=(10, 14), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t, tau_j[:, j], lw=0.8, label="tau_J (measured, incl. gravity)")
        if tau_cmd is not None:
            ax.plot(t, tau_cmd[:, j], lw=0.8, alpha=0.7,
                    label="tau (commanded, excl. gravity)")
        ax.axhline(TAU_LIMITS[j], color="r", ls="--", lw=0.8)
        ax.axhline(-TAU_LIMITS[j], color="r", ls="--", lw=0.8)
        ax.set_ylabel(f"j{j+1} [Nm]")
        ax.set_title(f"j{j+1}: max|tau_J|={abs_max[j]:.2f} Nm "
                     f"({100.0 * frac[j]:.1f}% of {TAU_LIMITS[j]:.0f})",
                     fontsize=9)
        if j == 0:
            ax.legend(fontsize=8, loc="upper right")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(csv_path.name)
    fig.tight_layout()

    out = Path(args.out).expanduser().resolve() if args.out \
        else csv_path.with_name(csv_path.stem + "_tau.png")
    fig.savefig(out, dpi=150)
    print(f"\nwrote figure: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
