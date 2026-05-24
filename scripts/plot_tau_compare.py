#!/usr/bin/env python3
"""Plot and summarize sim-vs-real J^T f_task torque comparison."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize tau_compare.csv diagnostics.")
    parser.add_argument("--csv", required=True, help="Path to *_tau_compare.csv")
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Default: <csv-dir>/tau_compare_<csv-stem>/",
    )
    parser.add_argument("--save", action="store_true")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--dpi", type=int, default=140)
    return parser.parse_args()


def ensure_columns(df: pd.DataFrame, required: list[str]) -> None:
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")


def _joint_cols(prefix: str) -> list[str]:
    return [f"{prefix}{i}" for i in range(1, 8)]


def print_summary_table(df: pd.DataFrame) -> None:
    real_cols = _joint_cols("tau_real_jt")
    sim_cols = _joint_cols("tau_sim_jt")
    dtau_cols = _joint_cols("dtau")

    tau_real = df[real_cols].to_numpy(dtype=np.float64)
    tau_sim = df[sim_cols].to_numpy(dtype=np.float64)
    dtau = df[dtau_cols].to_numpy(dtype=np.float64)

    tau_real_rms = np.sqrt(np.mean(tau_real**2, axis=0))
    tau_sim_rms = np.sqrt(np.mean(tau_sim**2, axis=0))
    dtau_rms = np.sqrt(np.mean(dtau**2, axis=0))
    dtau_max = np.max(np.abs(dtau), axis=0)

    print("-" * 76)
    print(f"{'joint':<8}{'tau_real_rms':>16}{'tau_sim_rms':>16}{'|dtau|_rms':>16}{'|dtau|_max':>16}")
    print("-" * 76)
    for j in range(7):
        print(
            f"{j+1:<8}"
            f"{tau_real_rms[j]:16.6f}"
            f"{tau_sim_rms[j]:16.6f}"
            f"{dtau_rms[j]:16.6f}"
            f"{dtau_max[j]:16.6f}"
        )
    print("-" * 76)


def build_plots(df: pd.DataFrame, out_dir: Path, save: bool, show: bool, dpi: int) -> None:
    if not show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = df["t_s"].to_numpy(dtype=np.float64)
    f_task = df[[f"f_{n}" for n in ("fx", "fy", "fz", "mx", "my", "mz")]].to_numpy(dtype=np.float64)
    tau_real = df[_joint_cols("tau_real_jt")].to_numpy(dtype=np.float64)
    tau_sim = df[_joint_cols("tau_sim_jt")].to_numpy(dtype=np.float64)
    dtau = df[_joint_cols("dtau")].to_numpy(dtype=np.float64)

    dtau_rms = np.sqrt(np.mean(dtau**2, axis=0))
    dtau_max = np.max(np.abs(dtau), axis=0)

    if save:
        out_dir.mkdir(parents=True, exist_ok=True)

    # 1) tau overlays per joint
    fig, axes = plt.subplots(7, 1, figsize=(11, 14), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t, tau_real[:, j], color="C0", linewidth=1.2, label="tau_real_jt")
        ax.plot(t, tau_sim[:, j], color="C1", linewidth=1.2, alpha=0.9, label="tau_sim_jt")
        ax.set_ylabel(f"j{j+1} [Nm]")
        ax.set_title(f"joint {j+1}: |dtau|_rms={dtau_rms[j]:.4f} Nm, |dtau|_max={dtau_max[j]:.4f} Nm", fontsize=9)
        ax.grid(True, alpha=0.3)
        if j == 0:
            ax.legend(loc="upper right")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("tau comparison: real vs sim (J^T f_task)")
    fig.tight_layout()
    if save:
        path = out_dir / "tau_per_joint.png"
        fig.savefig(path, dpi=dpi)
        print(f"[plot_tau_compare] wrote {path}")
    if show:
        plt.show()
    plt.close(fig)

    # 2) dtau per joint
    fig, axes = plt.subplots(7, 1, figsize=(11, 12), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t, dtau[:, j], color="C3", linewidth=1.2)
        ax.axhline(0.0, color="k", linewidth=0.8, alpha=0.5)
        ax.set_ylabel(f"j{j+1} [Nm]")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("dtau = tau_sim_jt - tau_real_jt")
    fig.tight_layout()
    if save:
        path = out_dir / "dtau_per_joint.png"
        fig.savefig(path, dpi=dpi)
        print(f"[plot_tau_compare] wrote {path}")
    if show:
        plt.show()
    plt.close(fig)

    # 3) f_task components
    fig, axes = plt.subplots(6, 1, figsize=(11, 11), sharex=True)
    labels = ["fx", "fy", "fz", "mx", "my", "mz"]
    for i, ax in enumerate(axes):
        ax.plot(t, f_task[:, i], color="C2", linewidth=1.2)
        ax.set_ylabel(labels[i])
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("f_task components used for both sim and real")
    fig.tight_layout()
    if save:
        path = out_dir / "f_task_timeseries.png"
        fig.savefig(path, dpi=dpi)
        print(f"[plot_tau_compare] wrote {path}")
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    args = parse_args()
    csv_path = Path(args.csv).expanduser().resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)

    df = pd.read_csv(csv_path)
    ensure_columns(
        df,
        ["t_s"]
        + [f"f_{n}" for n in ("fx", "fy", "fz", "mx", "my", "mz")]
        + _joint_cols("tau_real_jt")
        + _joint_cols("tau_sim_jt")
        + _joint_cols("dtau"),
    )

    if not (args.save or args.show):
        args.save = True

    if args.out_dir is None:
        out_dir = csv_path.parent / f"tau_compare_{csv_path.stem}"
    else:
        out_dir = Path(args.out_dir).expanduser().resolve()

    print_summary_table(df)
    build_plots(df, out_dir, save=args.save, show=args.show, dpi=args.dpi)
    if args.save:
        print(f"[plot_tau_compare] figures saved under {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
