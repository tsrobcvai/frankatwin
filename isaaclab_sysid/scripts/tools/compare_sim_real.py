#!/usr/bin/env python3
"""Compare a real cart_impedance.py log against its IsaacLab sim replay.

For each subplot we overlay three traces:

  * target  - `x_des_*` / `quat_des_*` (identical in both CSVs by construction)
  * real    - `x_*`     / `quat_*`     from the real-side CSV
  * sim     - `x_*`     / `quat_*`     from the sim CSV written by
              `isaaclab_sysid/scripts/tools/replay_python_csv_sim.py`

Both CSVs follow the schema documented in `docs/data_format.md`.

Besides the figures it scores the pair: sim against real at the end effector
(position RMSE per axis and 3-D, orientation RMSE) and per joint (q and dq
RMSE), plus each side's tracking error against the target. The numbers are
printed and, with `--save`, written to `metrics.json` next to the PNGs
(`compute_metrics` defines every field).

Runs on the SIM machine, next to the replay that wrote the sim CSV:
`isaaclab_sysid/install_into_isaaclab.sh` copies it into `<IsaacLab>/scripts/tools/`.
It needs only numpy, pandas and matplotlib -- no Isaac Sim -- so plain `python`
in the IsaacLab env is enough. `apply_sysid_params.py --invoke-replay` calls
`compare()` right after the replay, so the usual way to get here is that one
command; run this script directly to redo the figures or to compare any other
pair of CSVs.

Typical usage (from the IsaacLab root):

    python scripts/tools/compare_sim_real.py \\
        --real-csv data/<run>.csv \\
        --sim-csv  data/<run>_sim_sysid.csv \\
        --save --show
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd


POS_COLUMNS = ["x_x", "x_y", "x_z"]
POS_DES_COLUMNS = ["x_des_x", "x_des_y", "x_des_z"]
QUAT_COLUMNS = ["quat_x", "quat_y", "quat_z", "quat_w"]
QUAT_DES_COLUMNS = ["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]
JOINT_POS_COLUMNS = [f"q{i}" for i in range(1, 8)]
JOINT_VEL_COLUMNS = [f"dq{i}" for i in range(1, 8)]
METRICS_FILENAME = "metrics.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare sim vs real EE trajectories.")
    parser.add_argument("--real-csv", required=True, help="Real CSV path (cart_impedance.py log).")
    parser.add_argument("--sim-csv", required=True, help="Sim replay CSV path.")
    parser.add_argument("--real-sidecar", default=None, help="Optional real sidecar JSON (for stats header).")
    parser.add_argument("--sim-sidecar", default=None, help="Optional sim sidecar JSON (for stats header).")
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Directory for PNG outputs. Defaults to '<sim-csv-dir>/compare_<sim-stem>'.",
    )
    parser.add_argument("--save", action="store_true",
                        help="Write the PNGs and metrics.json to --out-dir (the default when --show is not given).")
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


def _rms(arr: np.ndarray, axis=None):
    return np.sqrt(np.mean(np.square(arr), axis=axis))


def _pos_block(err: np.ndarray) -> dict:
    """RMS and max of a (T, 3) position error [m]: per axis and 3-D (its norm)."""
    norm = np.linalg.norm(err, axis=1)
    rmse, peak = _rms(err, axis=0), np.max(np.abs(err), axis=0)
    return {
        "rmse_m": {"x": float(rmse[0]), "y": float(rmse[1]), "z": float(rmse[2]), "3d": float(_rms(norm))},
        "max_abs_m": {"x": float(peak[0]), "y": float(peak[1]), "z": float(peak[2]), "3d": float(np.max(norm))},
    }


def _ori_block(theta: np.ndarray) -> dict:
    return {"rmse_rad": float(_rms(theta)), "max_rad": float(np.max(theta))}


def compute_metrics(
    t: np.ndarray,
    x_des: np.ndarray,
    x_real: np.ndarray,
    x_sim: np.ndarray,
    quat_des: np.ndarray,
    quat_real: np.ndarray,
    quat_sim: np.ndarray,
    q_real: np.ndarray | None = None,
    q_sim: np.ndarray | None = None,
    dq_real: np.ndarray | None = None,
    dq_sim: np.ndarray | None = None,
) -> dict:
    """Score a sim replay against the real run it replays. All arrays share `t`.

    `ee` and `joints` are sim against real, the sim-to-real gap:

    * ``ee.pos``: `x_sim - x_real` [m] -- RMSE / max per axis and ``3d``, the
      RMS / max of the error's norm;
    * ``ee.ori``: shortest-path angle between the sim and the real quaternion [rad];
    * ``joints.pos_rmse_rad`` / ``vel_rmse_rad_s``: `q_sim - q_real` per joint;
      ``pos_rmse_all_rad`` over all seven at once and ``pos_mse_all_rad2`` its
      square (the quantity the fit minimises, without its weights);
      ``pos_rmse_pct_of_motion``: per joint, in percent of the range the real
      joint swept in this run (max - min).

    ``tracking`` is each side against the target both were given -- how far the
    impedance controller lags, not the sim-to-real gap.
    """
    metrics: dict = {
        "num_samples": int(len(t)),
        "duration_s": float(t[-1] - t[0]) if len(t) else 0.0,
        "ee": {
            "pos": _pos_block(x_sim - x_real),
            "ori": _ori_block(quat_theta_xyzw(quat_sim, quat_real)),
        },
        "joints": None,
        "tracking": {
            "real": {"pos": _pos_block(x_des - x_real), "ori": _ori_block(quat_theta_xyzw(quat_real, quat_des))},
            "sim": {"pos": _pos_block(x_des - x_sim), "ori": _ori_block(quat_theta_xyzw(quat_sim, quat_des))},
        },
    }
    if q_real is not None and q_sim is not None:
        err = q_sim - q_real
        rmse = _rms(err, axis=0)
        motion = np.max(q_real, axis=0) - np.min(q_real, axis=0)
        joints = {
            "pos_rmse_rad": rmse.tolist(),
            "pos_max_abs_rad": np.max(np.abs(err), axis=0).tolist(),
            "pos_rmse_all_rad": float(_rms(err)),
            "pos_mse_all_rad2": float(np.mean(np.square(err))),
            "pos_rmse_pct_of_motion": (100.0 * rmse / np.clip(motion, 1e-12, None)).tolist(),
            "real_motion_range_rad": motion.tolist(),
        }
        if dq_real is not None and dq_sim is not None:
            derr = dq_sim - dq_real
            joints["vel_rmse_rad_s"] = _rms(derr, axis=0).tolist()
            joints["vel_rmse_all_rad_s"] = float(_rms(derr))
        metrics["joints"] = joints
    return metrics


def print_summary(metrics: dict) -> None:
    """The numbers of `compute_metrics`, as a table."""
    ee, real, sim = metrics["ee"], metrics["tracking"]["real"], metrics["tracking"]["sim"]
    print("-" * 70)
    print(f"{'metric':<32}{'real':>12}{'sim':>12}{'sim - real':>14}")
    print("-" * 70)
    for key, label in (("rmse_m", "rms"), ("max_abs_m", "max")):
        for axis in ("x", "y", "z", "3d"):
            print(
                f"{f'err_pos_{label}_{axis} [mm]':<32}{real['pos'][key][axis]*1000:12.4f}"
                f"{sim['pos'][key][axis]*1000:12.4f}{ee['pos'][key][axis]*1000:14.4f}"
            )
    for key, label in (("rmse_rad", "theta_rms"), ("max_rad", "theta_max")):
        print(
            f"{f'{label} [mrad]':<32}{real['ori'][key]*1000:12.4f}"
            f"{sim['ori'][key]*1000:12.4f}{ee['ori'][key]*1000:14.4f}"
        )
    print("  real / sim: error against the target; sim - real: the sim-to-real gap")
    joints = metrics["joints"]
    if joints is not None:
        print("-" * 70)
        print(f"{'q_sim - q_real':<24}" + "".join(f"{f'j{j}':>9}" for j in range(1, 8)) + f"{'all':>9}")
        print(f"{'rmse [mrad]':<24}" + "".join(f"{v*1000:9.2f}" for v in joints["pos_rmse_rad"])
              + f"{joints['pos_rmse_all_rad']*1000:9.2f}")
        print(f"{'max [mrad]':<24}" + "".join(f"{v*1000:9.2f}" for v in joints["pos_max_abs_rad"]))
        print(f"{'rmse [% of motion]':<24}" + "".join(f"{v:9.2f}" for v in joints["pos_rmse_pct_of_motion"]))
        if "vel_rmse_rad_s" in joints:
            print(f"{'dq rmse [mrad/s]':<24}" + "".join(f"{v*1000:9.2f}" for v in joints["vel_rmse_rad_s"])
                  + f"{joints['vel_rmse_all_rad_s']*1000:9.2f}")
        print(f"joint-position MSE [rad^2]: {joints['pos_mse_all_rad2']:.4e}")
    print(f"duration [s]: {metrics['duration_s']:.3f}, samples: {metrics['num_samples']}")
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


def compare(
    real_csv,
    sim_csv,
    *,
    real_sidecar=None,
    sim_sidecar=None,
    out_dir=None,
    save: bool = True,
    show: bool = False,
    dpi: int = 140,
    extra: dict | None = None,
) -> dict:
    """Score `sim_csv` against `real_csv`, print the table, draw the figures.

    With `save` the PNGs and ``metrics.json`` go to `out_dir` (default
    ``<sim-csv-dir>/compare_<sim-stem>``). Returns the metrics: the fields of
    :func:`compute_metrics` plus the file paths, ``out_dir`` / ``metrics_path``
    (None without `save`) and whatever `extra` holds (``apply_sysid_params.py``
    puts the parameter file there).
    """
    real_csv = Path(real_csv).expanduser().resolve()
    sim_csv = Path(sim_csv).expanduser().resolve()
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
    q_real_arr = q_sim_arr = dq_real_arr = dq_sim_arr = None
    if have_joints:
        q_real_arr = real[JOINT_POS_COLUMNS].to_numpy()
        q_sim_arr = align_to_real_time(t_real, t_sim, sim[JOINT_POS_COLUMNS].to_numpy())
    if have_joint_vel:
        dq_real_arr = real[JOINT_VEL_COLUMNS].to_numpy()
        dq_sim_arr = align_to_real_time(t_real, t_sim, sim[JOINT_VEL_COLUMNS].to_numpy())

    print_sidecar_header(
        Path(real_sidecar) if real_sidecar else None,
        Path(sim_sidecar) if sim_sidecar else None,
    )
    metrics = compute_metrics(
        t_real, x_des_real, x_real, x_sim, quat_des_real, quat_real, quat_sim,
        q_real_arr, q_sim_arr, dq_real_arr, dq_sim_arr,
    )
    print_summary(metrics)

    if out_dir is None:
        out_dir = sim_csv.parent / f"compare_{sim_csv.stem}"
    else:
        out_dir = Path(out_dir).expanduser().resolve()

    save_paths = None
    if save:
        out_dir.mkdir(parents=True, exist_ok=True)
        save_paths = {
            "pos": out_dir / "position_timeseries.png",
            "quat": out_dir / "quaternion_timeseries.png",
            "theta": out_dir / "orientation_theta.png",
            "traj3d": out_dir / "traj3d.png",
            "joints": out_dir / "joint_absolute_values_7x2.png",
            "joint_err": out_dir / "joint_error_timeseries.png",
        }

    if not show:
        import matplotlib

        matplotlib.use("Agg")

    make_position_figure(
        t_real, x_des_real, x_real, x_sim,
        save_paths["pos"] if save_paths else None, show, dpi,
    )
    make_quaternion_figure(
        t_real, quat_des_real, quat_real, quat_sim,
        save_paths["quat"] if save_paths else None, show, dpi,
    )
    make_theta_figure(
        t_real, theta_real, theta_sim,
        save_paths["theta"] if save_paths else None, show, dpi,
    )
    make_traj3d_figure(
        x_des_real, x_real, x_sim,
        save_paths["traj3d"] if save_paths else None, show, dpi,
    )
    if have_joints:
        make_joints_figure(
            t_real, q_real_arr, q_sim_arr,
            dq_real_arr if have_joint_vel else np.zeros_like(q_real_arr),
            dq_sim_arr if have_joint_vel else np.zeros_like(q_real_arr),
            save_paths["joints"] if save_paths else None, show, dpi,
        )
        make_joint_error_figure(
            t_real, q_real_arr, q_sim_arr,
            save_paths["joint_err"] if save_paths else None, show, dpi,
        )

    metrics = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "real_csv": str(real_csv),
        "sim_csv": str(sim_csv),
        "real_sidecar": str(Path(real_sidecar).expanduser().resolve()) if real_sidecar else None,
        "sim_sidecar": str(Path(sim_sidecar).expanduser().resolve()) if sim_sidecar else None,
        **(extra or {}),
        **metrics,
        "out_dir": str(out_dir) if save else None,
        "metrics_path": str(out_dir / METRICS_FILENAME) if save else None,
    }
    if save:
        (out_dir / METRICS_FILENAME).write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        print(f"[compare] wrote {out_dir / METRICS_FILENAME}")
        print(f"[compare] figures saved under {out_dir}")
    return metrics


def main() -> int:
    args = parse_args()
    # Default: save next to the sim csv if neither flag is set.
    compare(
        args.real_csv, args.sim_csv,
        real_sidecar=args.real_sidecar, sim_sidecar=args.sim_sidecar,
        out_dir=args.out_dir, save=args.save or not args.show, show=args.show, dpi=args.dpi,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
