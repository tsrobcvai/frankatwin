#!/usr/bin/env python3
"""Plot Step5/Step5b Cartesian tracking logs."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Visualize Step5b Cartesian sweep logs: ideal vs actual EE trajectory "
            "and tracking errors."
        )
    )
    parser.add_argument(
        "csv_path",
        nargs="?",
        default=None,
        help="Path to CSV log. If omitted, use latest data/step5b_*.csv",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save PNG figures next to CSV.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively.",
    )
    return parser.parse_args()


def find_latest_step5b_csv(repo_root: Path) -> Path:
    data_dir = repo_root / "data"
    candidates = sorted(data_dir.glob("step5b_*.csv"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"No step5b CSV found under {data_dir}")
    return candidates[-1]


def ensure_columns(df: pd.DataFrame, required: list[str]) -> None:
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")


def build_plots(df: pd.DataFrame, csv_path: Path, save: bool, show: bool) -> None:
    # Only import matplotlib after deciding backend.
    if not show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = df["t_s"].to_numpy()
    x = df[["x_x", "x_y", "x_z"]].to_numpy()
    x_des = df[["x_des_x", "x_des_y", "x_des_z"]].to_numpy()
    e_pos = df[["e_x", "e_y", "e_z"]].to_numpy()
    e_pos_norm = np.linalg.norm(e_pos, axis=1)
    e_pos_inf = np.max(np.abs(e_pos), axis=1)
    e_ori = df[["e_ox", "e_oy", "e_oz"]].to_numpy()
    e_ori_norm = np.linalg.norm(e_ori, axis=1)
    quat = df[["quat_x", "quat_y", "quat_z", "quat_w"]].to_numpy()
    quat_des = df[["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]].to_numpy()
    # Align actual quaternion sign to desired so q vs -q ambiguity does not
    # cause visual jumps in the timeseries comparison.
    flip = np.sign(np.einsum("ij,ij->i", quat, quat_des))
    flip[flip == 0] = 1.0
    quat_aligned = quat * flip[:, None]

    stem = csv_path.with_suffix("")
    out_files = [
        stem.with_name(stem.name + "_traj3d.png"),
        stem.with_name(stem.name + "_position_timeseries.png"),
        stem.with_name(stem.name + "_position_error_norms.png"),
        stem.with_name(stem.name + "_orientation_error_norms.png"),
        stem.with_name(stem.name + "_quaternion_timeseries.png"),
    ]

    # Figure 1: 3D trajectory
    fig1 = plt.figure(figsize=(8.5, 6.5))
    ax3d = fig1.add_subplot(111, projection="3d")
    ax3d.plot(x_des[:, 0], x_des[:, 1], x_des[:, 2], label="x_des", linewidth=2.0)
    ax3d.plot(
        x[:, 0],
        x[:, 1],
        x[:, 2],
        "--",
        label="x_actual",
        linewidth=1.5,
    )
    ax3d.scatter(
        [x_des[0, 0]],
        [x_des[0, 1]],
        [x_des[0, 2]],
        marker="o",
        s=45,
        label="anchor",
    )
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title("Step5b EE Trajectory: Desired vs Actual")
    ax3d.legend(loc="best")
    fig1.tight_layout()

    # Figure 2: position timeseries (desired vs actual only)
    fig2, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axis_names = ["x", "y", "z"]
    for i, axis_name in enumerate(axis_names):
        ax = axes[i]
        ax.plot(t, x_des[:, i], label=f"{axis_name}_des", linewidth=1.8)
        ax.plot(t, x[:, i], "--", label=f"{axis_name}_actual", linewidth=1.3)
        ax.set_ylabel(f"{axis_name} (m)")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("time (s)")
    fig2.suptitle("Step5b Position Tracking")
    fig2.tight_layout()

    # Figure 3: position error norms
    fig3, ax3 = plt.subplots(figsize=(10, 4.5))
    ax3.plot(t, e_pos_norm, label="||e_pos||_2")
    ax3.plot(t, e_pos_inf, label="|e_pos|_inf")
    ax3.set_xlabel("time (s)")
    ax3.set_ylabel("error (m)")
    ax3.set_title("Step5b Position Error Norms")
    ax3.grid(True, alpha=0.25)
    ax3.legend(loc="best")
    fig3.tight_layout()

    # Figure 4: orientation error norms + components
    fig4, ax4 = plt.subplots(figsize=(10, 4.8))
    ax4.plot(t, e_ori_norm, label="||e_ori||_2", linewidth=2.0)
    ax4.plot(t, e_ori[:, 0], "--", label="e_ox", alpha=0.75)
    ax4.plot(t, e_ori[:, 1], "--", label="e_oy", alpha=0.75)
    ax4.plot(t, e_ori[:, 2], "--", label="e_oz", alpha=0.75)
    ax4.set_xlabel("time (s)")
    ax4.set_ylabel("error (rad)")
    ax4.set_title("Step5b Orientation Error")
    ax4.grid(True, alpha=0.25)
    ax4.legend(loc="best")
    fig4.tight_layout()

    # Figure 5: quaternion timeseries (desired vs actual, sign-aligned)
    fig5, axes5 = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    quat_labels = ["x", "y", "z", "w"]
    for i, comp in enumerate(quat_labels):
        ax = axes5[i]
        ax.plot(t, quat_des[:, i], label=f"q_des.{comp}", linewidth=1.8)
        ax.plot(t, quat_aligned[:, i], "--", label=f"q_actual.{comp}", linewidth=1.3)
        ax.set_ylabel(f"q.{comp}")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right")
    axes5[-1].set_xlabel("time (s)")
    fig5.suptitle("Step5b Quaternion Tracking (sign-aligned)")
    fig5.tight_layout()

    if save:
        for fig, out_path in zip((fig1, fig2, fig3, fig4, fig5), out_files):
            fig.savefig(out_path, dpi=140)
        print("[plot_step5] Saved figures:")
        for out_path in out_files:
            print(f"  - {out_path}")

    if show:
        plt.show()
    else:
        plt.close("all")

    pos_rms = np.sqrt(np.mean(e_pos**2, axis=0))
    pos_max_abs = np.max(np.abs(e_pos), axis=0)
    ori_rms = np.sqrt(np.mean(e_ori**2, axis=0))
    ori_max_abs = np.max(np.abs(e_ori), axis=0)
    print(f"[plot_step5] CSV: {csv_path}")
    print(
        "[plot_step5] Position RMS xyz (m): "
        f"{pos_rms[0]:.6f} {pos_rms[1]:.6f} {pos_rms[2]:.6f}"
    )
    print(
        "[plot_step5] Position max |e| xyz (m): "
        f"{pos_max_abs[0]:.6f} {pos_max_abs[1]:.6f} {pos_max_abs[2]:.6f}"
    )
    print(
        "[plot_step5] Position max |e|_inf (m): " f"{np.max(e_pos_inf):.6f}"
    )
    print(
        "[plot_step5] Orientation RMS xyz (rad): "
        f"{ori_rms[0]:.6f} {ori_rms[1]:.6f} {ori_rms[2]:.6f}"
    )
    print(
        "[plot_step5] Orientation max |e| xyz (rad): "
        f"{ori_max_abs[0]:.6f} {ori_max_abs[1]:.6f} {ori_max_abs[2]:.6f}"
    )
    print(
        "[plot_step5] Orientation max ||e||_2 (rad): "
        f"{np.max(e_ori_norm):.6f}"
    )


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]

    if args.csv_path is None:
        csv_path = find_latest_step5b_csv(repo_root)
    else:
        csv_path = Path(args.csv_path).expanduser().resolve()
    if not csv_path.exists():
        print(f"[plot_step5] CSV not found: {csv_path}", file=sys.stderr)
        return 2

    if not args.save and not args.show:
        # Useful default for headless runs and artifacts.
        args.save = True

    df = pd.read_csv(csv_path)
    ensure_columns(
        df,
        [
            "t_s",
            "x_x",
            "x_y",
            "x_z",
            "x_des_x",
            "x_des_y",
            "x_des_z",
            "e_x",
            "e_y",
            "e_z",
            "e_ox",
            "e_oy",
            "e_oz",
            "quat_x",
            "quat_y",
            "quat_z",
            "quat_w",
            "quat_des_x",
            "quat_des_y",
            "quat_des_z",
            "quat_des_w",
        ],
    )
    build_plots(df, csv_path, save=args.save, show=args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
