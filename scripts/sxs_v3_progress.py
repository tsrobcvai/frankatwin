#!/usr/bin/env python3
"""Side-by-side overlay for the in-progress v3 sysid (iter 17 snapshot).

Layout: 3 columns × 3 rows of EE position (x/y/z), one column per sim config
(baseline / v2 / v3 in-progress). Each panel overlays target + real + sim.

Also produces a 7×3 joint-position panel for the same configs when all
required CSVs are present.
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


def interp_mat(t_real: np.ndarray, t_sim: np.ndarray, mat: np.ndarray) -> np.ndarray:
    if len(t_real) == len(t_sim) and np.allclose(t_real, t_sim, atol=1e-4):
        return mat
    out = np.empty((len(t_real), mat.shape[1]), dtype=np.float64)
    for c in range(mat.shape[1]):
        out[:, c] = np.interp(t_real, t_sim, mat[:, c])
    return out


def rms(real: np.ndarray, sim: np.ndarray) -> float:
    return float(np.sqrt(np.mean((real - sim) ** 2)))


def sign_align_quat(quat: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Flip sign of each row of `quat` so it lies on the same hemisphere as `ref`."""
    dot = np.einsum("ij,ij->i", quat, ref)
    flip = np.where(dot < 0.0, -1.0, 1.0)
    return quat * flip[:, None]


def normalize_quat(quat: np.ndarray) -> np.ndarray:
    norm = np.clip(np.linalg.norm(quat, axis=-1, keepdims=True), 1e-12, None)
    return quat / norm


def quat_theta_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Shortest-path angle between two unit quaternion streams (xyzw)."""
    dot = np.clip(np.abs(np.einsum("ij,ij->i", q1, q2)), 0.0, 1.0)
    return 2.0 * np.arccos(dot)


QUAT_COLS = ["quat_x", "quat_y", "quat_z", "quat_w"]
QUAT_DES_COLS = ["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real-csv", required=True)
    ap.add_argument("--sim-csvs", nargs="+", required=True,
                    help="One CSV per config (baseline, v2, v3, ...).")
    ap.add_argument("--labels", nargs="+", required=True,
                    help="Display labels, must match --sim-csvs count.")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--title-suffix", default="")
    args = ap.parse_args()

    if len(args.sim_csvs) != len(args.labels):
        raise SystemExit("--sim-csvs and --labels must have the same length")

    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    real = pd.read_csv(args.real_csv)
    sims = [(lbl, pd.read_csv(csv)) for lbl, csv in zip(args.labels, args.sim_csvs)]
    t_real = real["t_s"].to_numpy()
    ncols = len(sims)

    fig, axes = plt.subplots(3, ncols, figsize=(5.0 * ncols, 9), sharex=True, squeeze=False)
    rms_pos = {}
    for col, (lbl, sim) in enumerate(sims):
        t_sim = sim["t_s"].to_numpy()
        rms_pos[lbl] = []
        for row, axis in enumerate("xyz"):
            ax = axes[row, col]
            real_v = real[f"x_{axis}"].to_numpy()
            sim_v = interp(t_real, t_sim, sim[f"x_{axis}"].to_numpy())
            ax.plot(t_real, real[f"x_des_{axis}"], "--", color="gray", lw=1.0, label="target")
            ax.plot(t_real, real_v, "-", color="C0", lw=1.1, label="real")
            ax.plot(t_real, sim_v, "-", color="C1", lw=1.1, alpha=0.85, label="sim")
            ax.grid(True, alpha=0.3)
            err_mm = 1000.0 * rms(real_v, sim_v)
            rms_pos[lbl].append(err_mm)
            ax.text(0.02, 0.95, f"|sim−real| RMS = {err_mm:.2f} mm",
                    transform=ax.transAxes, va="top", fontsize=8,
                    bbox=dict(facecolor="white", alpha=0.7, lw=0))
            if col == 0:
                ax.set_ylabel(f"{axis} [m]")
            if row == 0:
                ax.set_title(lbl, fontsize=11)
            if row == 0 and col == 0:
                ax.legend(loc="upper right", fontsize=8)
            if row == 2:
                ax.set_xlabel("t [s]")
    fig.suptitle(f"EE position: real vs sim {args.title_suffix}".strip(), fontsize=12)
    fig.tight_layout()
    fig.savefig(out_dir / "sxs_position.png", dpi=140)
    plt.close(fig)
    print(f"wrote {out_dir / 'sxs_position.png'}")

    have_joints = all(f"q{i}" in s.columns for _, s in sims for i in range(1, 8))
    have_joints = have_joints and all(f"q{i}" in real.columns for i in range(1, 8))
    if have_joints:
        fig, axes = plt.subplots(7, ncols, figsize=(5.0 * ncols, 16), sharex=True, squeeze=False)
        for col, (lbl, sim) in enumerate(sims):
            t_sim = sim["t_s"].to_numpy()
            for j in range(7):
                ax = axes[j, col]
                rv = real[f"q{j+1}"].to_numpy()
                sv = interp(t_real, t_sim, sim[f"q{j+1}"].to_numpy())
                ax.plot(t_real, rv, "-", color="C0", lw=1.0, label="real")
                ax.plot(t_real, sv, "-", color="C1", lw=1.0, alpha=0.85, label="sim")
                ax.grid(True, alpha=0.3)
                rmse_rad = rms(rv, sv)
                ax.text(0.02, 0.95, f"RMS={rmse_rad*1000:.2f} mrad",
                        transform=ax.transAxes, va="top", fontsize=8,
                        bbox=dict(facecolor="white", alpha=0.7, lw=0))
                if col == 0:
                    ax.set_ylabel(f"j{j+1}\nq [rad]", fontsize=9)
                if j == 0:
                    ax.set_title(lbl, fontsize=11)
                if j == 0 and col == 0:
                    ax.legend(loc="upper right", fontsize=8)
                if j == 6:
                    ax.set_xlabel("t [s]")
        fig.suptitle(f"Joint q: real vs sim {args.title_suffix}".strip(), fontsize=12)
        fig.tight_layout()
        fig.savefig(out_dir / "sxs_joints.png", dpi=140)
        plt.close(fig)
        print(f"wrote {out_dir / 'sxs_joints.png'}")

    have_quat = all(c in real.columns for c in QUAT_COLS + QUAT_DES_COLS) and all(
        all(c in s.columns for c in QUAT_COLS) for _, s in sims
    )

    rms_ori = {}
    if have_quat:
        quat_des = normalize_quat(real[QUAT_DES_COLS].to_numpy())
        quat_real_raw = normalize_quat(real[QUAT_COLS].to_numpy())
        quat_real = sign_align_quat(quat_real_raw, quat_des)
        theta_real_target = quat_theta_xyzw(quat_real, quat_des)

        fig, axes = plt.subplots(4, ncols, figsize=(5.0 * ncols, 11), sharex=True, squeeze=False)
        for col, (lbl, sim) in enumerate(sims):
            t_sim = sim["t_s"].to_numpy()
            q_sim_raw = normalize_quat(
                interp_mat(t_real, t_sim, sim[QUAT_COLS].to_numpy())
            )
            q_sim = sign_align_quat(q_sim_raw, quat_des)
            for row, ch in enumerate(("qx", "qy", "qz", "qw")):
                ax = axes[row, col]
                ax.plot(t_real, quat_des[:, row], "--", color="gray", lw=1.0, label="target")
                ax.plot(t_real, quat_real[:, row], "-", color="C0", lw=1.1, label="real")
                ax.plot(t_real, q_sim[:, row], "-", color="C1", lw=1.1, alpha=0.85, label="sim")
                ax.grid(True, alpha=0.3)
                ch_rms = rms(quat_real[:, row], q_sim[:, row])
                ax.text(0.02, 0.95, f"|sim−real| RMS = {ch_rms:.4f}",
                        transform=ax.transAxes, va="top", fontsize=8,
                        bbox=dict(facecolor="white", alpha=0.7, lw=0))
                if col == 0:
                    ax.set_ylabel(ch)
                if row == 0:
                    ax.set_title(lbl, fontsize=11)
                if row == 0 and col == 0:
                    ax.legend(loc="upper right", fontsize=8)
                if row == 3:
                    ax.set_xlabel("t [s]")
        fig.suptitle(f"EE quaternion (xyzw, sign-aligned to target): real vs sim {args.title_suffix}".strip(), fontsize=12)
        fig.tight_layout()
        fig.savefig(out_dir / "sxs_quaternion.png", dpi=140)
        plt.close(fig)
        print(f"wrote {out_dir / 'sxs_quaternion.png'}")

        fig, axes = plt.subplots(1, ncols, figsize=(5.0 * ncols, 3.5), sharex=True, sharey=True, squeeze=False)
        for col, (lbl, sim) in enumerate(sims):
            t_sim = sim["t_s"].to_numpy()
            q_sim_raw = normalize_quat(
                interp_mat(t_real, t_sim, sim[QUAT_COLS].to_numpy())
            )
            q_sim = sign_align_quat(q_sim_raw, quat_des)
            theta_sim_target = quat_theta_xyzw(q_sim, quat_des)
            theta_sim_real = quat_theta_xyzw(q_sim, quat_real)
            ax = axes[0, col]
            ax.plot(t_real, theta_real_target * 1000.0, "-", color="C0", lw=1.1, label="real vs target")
            ax.plot(t_real, theta_sim_target * 1000.0, "-", color="C1", lw=1.1, alpha=0.85, label="sim vs target")
            ax.plot(t_real, theta_sim_real * 1000.0, "-", color="C3", lw=1.0, alpha=0.85, label="sim vs real")
            ax.grid(True, alpha=0.3)
            ax.set_title(lbl, fontsize=11)
            ax.set_xlabel("t [s]")
            if col == 0:
                ax.set_ylabel("|orientation error| [mrad]")
                ax.legend(loc="upper right", fontsize=8)
            theta_sim_real_rms_mrad = float(np.sqrt(np.mean(theta_sim_real ** 2)) * 1000.0)
            theta_sim_target_rms_mrad = float(np.sqrt(np.mean(theta_sim_target ** 2)) * 1000.0)
            theta_real_target_rms_mrad = float(np.sqrt(np.mean(theta_real_target ** 2)) * 1000.0)
            rms_ori[lbl] = {
                "sim_vs_real_rms_mrad": theta_sim_real_rms_mrad,
                "sim_vs_target_rms_mrad": theta_sim_target_rms_mrad,
                "real_vs_target_rms_mrad": theta_real_target_rms_mrad,
                "sim_vs_real_max_mrad": float(np.max(theta_sim_real) * 1000.0),
            }
            ax.text(
                0.02, 0.95,
                "RMS [mrad]:\n"
                f"  real vs target = {theta_real_target_rms_mrad:.2f}\n"
                f"  sim  vs target = {theta_sim_target_rms_mrad:.2f}\n"
                f"  sim  vs real   = {theta_sim_real_rms_mrad:.2f}",
                transform=ax.transAxes, va="top", fontsize=8,
                bbox=dict(facecolor="white", alpha=0.75, lw=0),
            )
        fig.suptitle(f"Shortest-path orientation angle {args.title_suffix}".strip(), fontsize=12)
        fig.tight_layout()
        fig.savefig(out_dir / "sxs_orientation_theta.png", dpi=140)
        plt.close(fig)
        print(f"wrote {out_dir / 'sxs_orientation_theta.png'}")

    summary = ["config,err_pos_rms_mm_x,err_pos_rms_mm_y,err_pos_rms_mm_z,err_pos_rms_mm_norm,"
               "sim_vs_real_theta_rms_mrad,sim_vs_target_theta_rms_mrad,real_vs_target_theta_rms_mrad"]
    for lbl, vals in rms_pos.items():
        norm = float(np.sqrt(sum(v * v for v in vals) / 3.0))
        ori = rms_ori.get(lbl, {})
        summary.append(
            f"{lbl},{vals[0]:.3f},{vals[1]:.3f},{vals[2]:.3f},{norm:.3f},"
            f"{ori.get('sim_vs_real_rms_mrad', float('nan')):.3f},"
            f"{ori.get('sim_vs_target_rms_mrad', float('nan')):.3f},"
            f"{ori.get('real_vs_target_rms_mrad', float('nan')):.3f}"
        )
    (out_dir / "summary.csv").write_text("\n".join(summary) + "\n")
    print(f"wrote {out_dir / 'summary.csv'}")
    print("\nposition RMS [mm] (X / Y / Z / norm)  |  theta RMS [mrad] (sim vs real / sim vs tgt / real vs tgt):")
    for s in summary[1:]:
        print("  " + s)


if __name__ == "__main__":
    main()
