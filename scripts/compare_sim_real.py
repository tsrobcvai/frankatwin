#!/usr/bin/env python3
"""Compare a real step5b log against the IsaacLab sim replay.

For each subplot we overlay three traces:

  * target  - `x_des_*` / `quat_des_*` (identical in both CSVs by construction)
  * real    - `x_*`     / `quat_*`     from the real-side step5b CSV
  * sim     - `x_*`     / `quat_*`     from the sim CSV written by
              `IsaacLab/scripts/tools/replay_real_step5b_sim.py`

Both CSVs must follow the schema documented in
`frankatwin/SIM2REAL_COMPARISON.md` (real §2.1, sim §5.4).

Typical usage:

    python scripts/compare_sim_real_step5b.py \\
        --real-csv data/step5b_20260524_120834.csv \\
        --sim-csv  data/step5b_20260524_120834_sim.csv \\
        --save --show
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd


POS_COLUMNS = ["x_x", "x_y", "x_z"]
POS_DES_COLUMNS = ["x_des_x", "x_des_y", "x_des_z"]
QUAT_COLUMNS = ["quat_x", "quat_y", "quat_z", "quat_w"]
QUAT_DES_COLUMNS = ["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]
JOINT_POS_COLUMNS = [f"q{i}" for i in range(1, 8)]
JOINT_VEL_COLUMNS = [f"dq{i}" for i in range(1, 8)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare sim vs real step5b EE trajectories.")
    parser.add_argument("--real-csv", required=True, help="Real step5b CSV path.")
    parser.add_argument("--sim-csv", required=True, help="Sim replay CSV path.")
    parser.add_argument("--real-sidecar", default=None, help="Optional real sidecar JSON (for stats header).")
    parser.add_argument("--sim-sidecar", default=None, help="Optional sim sidecar JSON (for stats header).")
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Directory for PNG outputs. Defaults to '<sim-csv-dir>/compare_<sim-stem>'.",
    )
    parser.add_argument("--save", action="store_true", help="Write PNGs to --out-dir.")
    parser.add_argument("--show", action="store_true", help="Display figures interactively.")
    parser.add_argument("--dpi", type=int, default=140)
    return parser.parse_args()


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = ["t_s", *POS_COLUMNS, *POS_DES_COLUMNS, *QUAT_COLUMNS, *QUAT_DES_COLUMNS]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"{path} missing required columns: {missing}")
    return df


def align_to_real_time(real_t: np.ndarray, sim_t: np.ndarray, sim_arr: np.ndarray) -> np.ndarray:
    """Linear interpolation of `sim_arr` (T_sim, D) onto `real_t`."""
    if len(real_t) == len(sim_t) and np.allclose(real_t, sim_t, atol=1e-4):
        return sim_arr
    out = np.empty((len(real_t), sim_arr.shape[1]), dtype=np.float64)
    for c in range(sim_arr.shape[1]):
        out[:, c] = np.interp(real_t, sim_t, sim_arr[:, c])
    return out


def sign_align_quat(quat: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Flip sign of each `quat` row so it lies on the same hemisphere as `ref`."""
    dot = np.einsum("ij,ij->i", quat, ref)
    flip = np.where(dot < 0.0, -1.0, 1.0)
    return quat * flip[:, None]


def quat_theta_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Shortest-path angle between two unit quaternion streams."""
    dot = np.clip(np.abs(np.einsum("ij,ij->i", q1, q2)), 0.0, 1.0)
    return 2.0 * np.arccos(dot)


def make_position_figure(t, x_des, x_real, x_sim, save_path: Path | None, show: bool, dpi: int):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, label, idx in zip(axes, ("x", "y", "z"), range(3)):
        ax.plot(t, x_des[:, idx], "--", color="gray", linewidth=1.5, label="target")
        ax.plot(t, x_real[:, idx], "-", color="C0", linewidth=1.2, label="real")
        ax.plot(t, x_sim[:, idx], "-", color="C1", linewidth=1.2, alpha=0.85, label="sim")
        ax.set_ylabel(f"{label} [m]")
        ax.grid(True, alpha=0.3)
        if idx == 0:
            ax.legend(loc="upper right")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("EE position: target vs real vs sim")
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=dpi)
        print(f"[compare] wrote {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def make_quaternion_figure(t, q_des, q_real, q_sim, save_path: Path | None, show: bool, dpi: int):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for ax, label, idx in zip(axes, ("qx", "qy", "qz", "qw"), range(4)):
        ax.plot(t, q_des[:, idx], "--", color="gray", linewidth=1.5, label="target")
        ax.plot(t, q_real[:, idx], "-", color="C0", linewidth=1.2, label="real")
        ax.plot(t, q_sim[:, idx], "-", color="C1", linewidth=1.2, alpha=0.85, label="sim")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        if idx == 0:
            ax.legend(loc="upper right")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("EE orientation quaternion (xyzw, sign-aligned to target)")
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=dpi)
        print(f"[compare] wrote {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def make_theta_figure(t, theta_real, theta_sim, save_path: Path | None, show: bool, dpi: int):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, theta_real, "-", color="C0", linewidth=1.2, label="real vs target")
    ax.plot(t, theta_sim, "-", color="C1", linewidth=1.2, alpha=0.85, label="sim vs target")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|orientation error| [rad]")
    ax.set_title("Shortest-path angle to target quaternion")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=dpi)
        print(f"[compare] wrote {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def make_joints_figure(
    t: np.ndarray,
    q_real: np.ndarray,
    q_sim: np.ndarray,
    dq_real: np.ndarray,
    dq_sim: np.ndarray,
    save_path: Path | None,
    show: bool,
    dpi: int,
):
    """7x2 grid: per-joint q (left) and dq (right), sim vs real."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(7, 2, figsize=(12, 14), sharex=True)
    for j in range(7):
        ax_q = axes[j, 0]
        ax_dq = axes[j, 1]
        ax_q.plot(t, q_real[:, j], color="C0", linewidth=1.0, label="real" if j == 0 else None)
        ax_q.plot(t, q_sim[:, j], color="C1", linewidth=1.0, alpha=0.85, label="sim" if j == 0 else None)
        ax_q.set_ylabel(f"joint {j+1}\nq [rad]", fontsize=9)
        ax_q.grid(True, alpha=0.3)
        ax_dq.plot(t, dq_real[:, j], color="C0", linewidth=1.0)
        ax_dq.plot(t, dq_sim[:, j], color="C1", linewidth=1.0, alpha=0.85)
        ax_dq.set_ylabel(f"dq [rad/s]", fontsize=9)
        ax_dq.grid(True, alpha=0.3)
        if j == 0:
            ax_q.set_title("Joint position q")
            ax_dq.set_title("Joint velocity dq")
            ax_q.legend(loc="upper right", fontsize=8)
    axes[-1, 0].set_xlabel("t [s]")
    axes[-1, 1].set_xlabel("t [s]")
    fig.suptitle("Per-joint sim vs real (7x2)")
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=dpi)
        print(f"[compare] wrote {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def make_joint_error_figure(
    t: np.ndarray,
    q_real: np.ndarray,
    q_sim: np.ndarray,
    save_path: Path | None,
    show: bool,
    dpi: int,
):
    """Single panel: (q_sim - q_real) per joint."""
    import matplotlib.pyplot as plt

    err = q_sim - q_real
    fig, ax = plt.subplots(figsize=(10, 5))
    for j in range(7):
        ax.plot(t, err[:, j] * 1000.0, linewidth=1.0, label=f"j{j+1}")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("q_sim - q_real [mrad]")
    ax.set_title("Per-joint sim-real position error")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", ncol=7, fontsize=8)
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=dpi)
        print(f"[compare] wrote {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def make_traj3d_figure(x_des, x_real, x_sim, save_path: Path | None, show: bool, dpi: int):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(x_des[:, 0], x_des[:, 1], x_des[:, 2], "--", color="gray", linewidth=1.2, label="target")
    ax.plot(x_real[:, 0], x_real[:, 1], x_real[:, 2], "-", color="C0", linewidth=1.0, label="real")
    ax.plot(x_sim[:, 0], x_sim[:, 1], x_sim[:, 2], "-", color="C1", linewidth=1.0, alpha=0.85, label="sim")
    ax.scatter(x_des[0, 0], x_des[0, 1], x_des[0, 2], color="black", s=20, label="anchor")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("EE trajectory (3D)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=dpi)
        print(f"[compare] wrote {save_path}")
    if show:
        plt.show()
    plt.close(fig)


def print_summary(
    t: np.ndarray,
    x_des: np.ndarray,
    x_real: np.ndarray,
    x_sim: np.ndarray,
    theta_real: np.ndarray,
    theta_sim: np.ndarray,
) -> None:
    def rms(arr: np.ndarray) -> np.ndarray:
        return np.sqrt(np.mean(arr**2, axis=0))

    pos_real_err = x_des - x_real
    pos_sim_err = x_des - x_sim
    pos_delta = x_sim - x_real

    print("-" * 70)
    print(f"{'metric':<32}{'real':>12}{'sim':>12}{'sim - real':>14}")
    print("-" * 70)
    for i, axis in enumerate("xyz"):
        print(
            f"err_pos_rms_{axis} [mm]                {rms(pos_real_err)[i]*1000:12.4f}"
            f"{rms(pos_sim_err)[i]*1000:12.4f}{rms(pos_delta)[i]*1000:14.4f}"
        )
    for i, axis in enumerate("xyz"):
        print(
            f"err_pos_max_{axis} [mm]                {np.max(np.abs(pos_real_err[:, i]))*1000:12.4f}"
            f"{np.max(np.abs(pos_sim_err[:, i]))*1000:12.4f}"
            f"{np.max(np.abs(pos_delta[:, i]))*1000:14.4f}"
        )
    print(
        f"theta_rms          [mrad]            "
        f"{np.sqrt(np.mean(theta_real**2))*1000:12.4f}"
        f"{np.sqrt(np.mean(theta_sim**2))*1000:12.4f}"
        f"{'-':>14}"
    )
    print(
        f"theta_max          [mrad]            "
        f"{np.max(theta_real)*1000:12.4f}"
        f"{np.max(theta_sim)*1000:12.4f}"
        f"{'-':>14}"
    )
    print(f"duration [s]: {t[-1]:.3f}, samples: {len(t)}")
    print("-" * 70)


def print_sidecar_header(real_sidecar: Path | None, sim_sidecar: Path | None) -> None:
    def safe_load(path: Path | None):
        if path is None:
            return None
        try:
            return json.loads(Path(path).read_text())
        except Exception as exc:  # pragma: no cover - defensive only
            print(f"[compare] WARN: failed to read {path}: {exc}", file=sys.stderr)
            return None

    real_meta = safe_load(real_sidecar)
    sim_meta = safe_load(sim_sidecar)
    if real_meta:
        args = real_meta.get("args", {})
        print(
            "[compare] real:  controller={ctrl}, kp_pos={kp_pos}, kp_ori={kp_ori}, axis={axis}, "
            "amp={amp}, freq={freq}".format(
                ctrl=real_meta.get("controller"),
                kp_pos=args.get("kp_pos"),
                kp_ori=args.get("kp_ori"),
                axis=args.get("axis"),
                amp=args.get("amp"),
                freq=args.get("freq"),
            )
        )
    if sim_meta:
        sim_block = sim_meta.get("sim", {})
        print(
            "[compare] sim:   controller={ctrl}, engine={engine}, integrator_hz={hz}, "
            "use_nullspace={ns}".format(
                ctrl=sim_meta.get("controller"),
                engine=sim_block.get("engine"),
                hz=sim_block.get("integrator_hz"),
                ns=sim_block.get("use_nullspace"),
            )
        )


def main() -> int:
    args = parse_args()

    real_csv = Path(args.real_csv).expanduser().resolve()
    sim_csv = Path(args.sim_csv).expanduser().resolve()
    if not real_csv.is_file():
        raise FileNotFoundError(real_csv)
    if not sim_csv.is_file():
        raise FileNotFoundError(sim_csv)

    real = load_csv(real_csv)
    sim = load_csv(sim_csv)

    t_real = real["t_s"].to_numpy()
    t_sim = sim["t_s"].to_numpy()

    x_des_real = real[POS_DES_COLUMNS].to_numpy()
    x_real = real[POS_COLUMNS].to_numpy()
    x_sim = align_to_real_time(t_real, t_sim, sim[POS_COLUMNS].to_numpy())

    quat_des_real = real[QUAT_DES_COLUMNS].to_numpy()
    quat_real = sign_align_quat(real[QUAT_COLUMNS].to_numpy(), quat_des_real)
    quat_sim_raw = align_to_real_time(t_real, t_sim, sim[QUAT_COLUMNS].to_numpy())
    quat_sim_raw /= np.clip(np.linalg.norm(quat_sim_raw, axis=1, keepdims=True), 1e-12, None)
    quat_sim = sign_align_quat(quat_sim_raw, quat_des_real)

    theta_real = quat_theta_xyzw(quat_real, quat_des_real)
    theta_sim = quat_theta_xyzw(quat_sim, quat_des_real)

    have_joints = all(c in real.columns for c in JOINT_POS_COLUMNS) and all(c in sim.columns for c in JOINT_POS_COLUMNS)
    have_joint_vel = all(c in real.columns for c in JOINT_VEL_COLUMNS) and all(c in sim.columns for c in JOINT_VEL_COLUMNS)
    if have_joints:
        q_real_arr = real[JOINT_POS_COLUMNS].to_numpy()
        q_sim_arr = align_to_real_time(t_real, t_sim, sim[JOINT_POS_COLUMNS].to_numpy())
    if have_joint_vel:
        dq_real_arr = real[JOINT_VEL_COLUMNS].to_numpy()
        dq_sim_arr = align_to_real_time(t_real, t_sim, sim[JOINT_VEL_COLUMNS].to_numpy())

    print_sidecar_header(
        Path(args.real_sidecar) if args.real_sidecar else None,
        Path(args.sim_sidecar) if args.sim_sidecar else None,
    )
    print_summary(t_real, x_des_real, x_real, x_sim, theta_real, theta_sim)

    if not (args.save or args.show):
        # Default: save next to sim csv if neither flag set, to mimic plot_step5.py.
        args.save = True

    if args.out_dir is None:
        out_dir = sim_csv.parent / f"compare_{sim_csv.stem}"
    else:
        out_dir = Path(args.out_dir).expanduser().resolve()

    save_paths = None
    if args.save:
        out_dir.mkdir(parents=True, exist_ok=True)
        save_paths = {
            "pos": out_dir / "position_timeseries.png",
            "quat": out_dir / "quaternion_timeseries.png",
            "theta": out_dir / "orientation_theta.png",
            "traj3d": out_dir / "traj3d.png",
            "joints": out_dir / "joint_absolute_values_7x2.png",
            "joint_err": out_dir / "joint_error_timeseries.png",
        }

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")

    make_position_figure(
        t_real, x_des_real, x_real, x_sim,
        save_paths["pos"] if save_paths else None, args.show, args.dpi,
    )
    make_quaternion_figure(
        t_real, quat_des_real, quat_real, quat_sim,
        save_paths["quat"] if save_paths else None, args.show, args.dpi,
    )
    make_theta_figure(
        t_real, theta_real, theta_sim,
        save_paths["theta"] if save_paths else None, args.show, args.dpi,
    )
    make_traj3d_figure(
        x_des_real, x_real, x_sim,
        save_paths["traj3d"] if save_paths else None, args.show, args.dpi,
    )
    if have_joints:
        make_joints_figure(
            t_real, q_real_arr, q_sim_arr,
            dq_real_arr if have_joint_vel else np.zeros_like(q_real_arr),
            dq_sim_arr if have_joint_vel else np.zeros_like(q_real_arr),
            save_paths["joints"] if save_paths else None, args.show, args.dpi,
        )
        make_joint_error_figure(
            t_real, q_real_arr, q_sim_arr,
            save_paths["joint_err"] if save_paths else None, args.show, args.dpi,
        )

    if save_paths:
        print(f"[compare] figures saved under {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
