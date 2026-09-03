# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Franka system identification via CMA-ES (task-impedance closed-loop replay).

Supports **multi-trajectory** joint fitting: pass ``--real_csv`` and
``--real_sidecar`` multiple times to optimise over a sum of per-trajectory
losses.  This is how v2 sysid lifts the under-identifiability of v1 (which
was trained on the slow 0.25 Hz step5b z-sin only) — step5c adds the
0.7-1.1 Hz acceleration content needed for armature / viscous / delay.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from datetime import datetime
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Franka system ID (CMA-ES) on step5b/step5c replay.")
parser.add_argument("--task", type=str, default="Isaac-FrankaTwin-Sysid-v0")
parser.add_argument("--num_envs", type=int, default=128, help="CMA population size.")
parser.add_argument(
    "--real_csv",
    type=str,
    required=True,
    action="append",
    help="Real CSV path. May be repeated for multi-trajectory joint fit (order must match --real_sidecar).",
)
parser.add_argument(
    "--real_sidecar",
    type=str,
    required=True,
    action="append",
    help="Real sidecar path. May be repeated; must align 1:1 with --real_csv.",
)
parser.add_argument(
    "--traj_weights",
    type=str,
    default=None,
    help="Optional comma-separated per-trajectory weights (default: 1.0 for all).",
)
parser.add_argument("--max_iter", type=int, default=80)
parser.add_argument("--sigma", type=float, default=0.3)
parser.add_argument("--save_interval", type=int, default=10)
parser.add_argument("--max_t_s", type=float, default=None, help="Optional trajectory truncation in seconds (applied to every CSV).")
parser.add_argument("--out_dir", type=str, default="logs/sysid_franka")
parser.add_argument("--w_q", type=float, default=1.0)
parser.add_argument("--w_dq", type=float, default=0.1)
parser.add_argument("--w_x", type=float, default=0.01)
parser.add_argument("--armature_min", type=float, default=0.0)
parser.add_argument("--armature_max", type=float, default=0.5)
parser.add_argument("--friction_min", type=float, default=0.0)
parser.add_argument("--friction_max", type=float, default=5.0)
parser.add_argument("--viscous_min", type=float, default=0.0)
parser.add_argument("--viscous_max", type=float, default=5.0)
parser.add_argument("--delay_max", type=int, default=4)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

import sys

sys.argv = [sys.argv[0]] + hydra_args
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab_tasks  # noqa: F401, E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


ARM_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
NUM_ARM_JOINTS = 7
NUM_PARAMS = NUM_ARM_JOINTS * 4 + 1  # armature, static, dyn_ratio, viscous, delay


class CMAES:
    """CMA-ES with [0,1] normalized latent variables mapped to real bounds."""

    def __init__(self, num_params: int, population_size: int, sigma: float, bounds: np.ndarray):
        from cmaes import CMA

        self.num_params = num_params
        self.population_size = population_size
        self.bounds = np.asarray(bounds, dtype=np.float64)
        self.optimizer = CMA(
            mean=np.full(num_params, 0.5, dtype=np.float64),
            sigma=float(sigma),
            population_size=int(population_size),
            bounds=np.column_stack([np.zeros(num_params), np.ones(num_params)]),
        )
        self._solutions = None

    def ask(self) -> np.ndarray:
        self._solutions = [self.optimizer.ask() for _ in range(self.population_size)]
        z = np.asarray(self._solutions, dtype=np.float64)  # normalized [0,1]
        lo = self.bounds[:, 0]
        hi = self.bounds[:, 1]
        return lo + z * (hi - lo)

    def tell(self, scores: np.ndarray):
        self.optimizer.tell(list(zip(self._solutions, scores.tolist())))

    @property
    def best_params(self) -> np.ndarray:
        z = self.optimizer._mean
        lo = self.bounds[:, 0]
        hi = self.bounds[:, 1]
        return lo + z * (hi - lo)


def build_bounds(args) -> np.ndarray:
    bounds = []
    bounds += [[args.armature_min, args.armature_max] for _ in range(NUM_ARM_JOINTS)]
    bounds += [[args.friction_min, args.friction_max] for _ in range(NUM_ARM_JOINTS)]  # mu_static
    bounds += [[0.0, 1.0] for _ in range(NUM_ARM_JOINTS)]  # dynamic ratio
    bounds += [[args.viscous_min, args.viscous_max] for _ in range(NUM_ARM_JOINTS)]
    bounds += [[0.0, float(args.delay_max)]]
    return np.asarray(bounds, dtype=np.float64)


def decode_params(params: np.ndarray) -> dict:
    return {
        "armature": params[0:7],
        "mu_static": params[7:14],
        "dynamic_ratio": params[14:21],
        "mu_dynamic": params[14:21] * params[7:14],
        "mu_viscous": params[21:28],
        "motor_delay_steps": int(round(float(params[28]))),
    }


def apply_params_to_envs(robot: Articulation, params_tensor: torch.Tensor, arm_joint_ids, num_joints: int, delay_max: int):
    """Apply per-env dynamic params to all envs in the batch."""
    device = params_tensor.device
    n_env = params_tensor.shape[0]
    env_ids = torch.arange(n_env, device=device)

    armature_full = torch.zeros((n_env, num_joints), device=device)
    static_fric_full = torch.zeros((n_env, num_joints), device=device)
    dynamic_fric_full = torch.zeros((n_env, num_joints), device=device)
    viscous_fric_full = torch.zeros((n_env, num_joints), device=device)

    armature = params_tensor[:, 0:7]
    static_fric = params_tensor[:, 7:14]
    dynamic_ratio = params_tensor[:, 14:21]
    viscous_fric = params_tensor[:, 21:28]
    dynamic_fric = dynamic_ratio * static_fric

    armature_full[:, arm_joint_ids] = armature
    static_fric_full[:, arm_joint_ids] = static_fric
    dynamic_fric_full[:, arm_joint_ids] = dynamic_fric
    viscous_fric_full[:, arm_joint_ids] = viscous_fric

    robot.write_joint_armature_to_sim(armature_full, env_ids=env_ids)
    robot.write_joint_friction_coefficient_to_sim(
        static_fric_full,
        joint_dynamic_friction_coeff=dynamic_fric_full,
        joint_viscous_friction_coeff=viscous_fric_full,
        env_ids=env_ids,
    )

    delay_int = torch.round(params_tensor[:, 28]).clamp(min=0, max=float(delay_max)).to(torch.int)
    for act_name in ("panda_arm1", "panda_arm2"):
        actuator = robot.actuators[act_name]
        actuator.positions_delay_buffer.set_time_lag(delay_int, env_ids)
        actuator.velocities_delay_buffer.set_time_lag(delay_int, env_ids)
        actuator.efforts_delay_buffer.set_time_lag(delay_int, env_ids)


def load_real_data(csv_path: Path, sidecar_path: Path, max_t_s: float | None, device: str):
    with open(sidecar_path, "r", encoding="utf-8") as f:
        sidecar = json.load(f)
    arr = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=np.float64)
    arr = np.atleast_1d(arr)
    if arr.size == 0:
        raise ValueError(f"Empty real csv: {csv_path}")

    t = arr["t_s"]
    if max_t_s is not None:
        keep = t <= float(max_t_s)
        arr = arr[keep]
        t = arr["t_s"]
    t = np.asarray(t, dtype=np.float64)

    q = np.column_stack([arr[f"q{i}"] for i in range(1, 8)]).astype(np.float32)
    dq = np.column_stack([arr[f"dq{i}"] for i in range(1, 8)]).astype(np.float32)
    x = np.column_stack((arr["x_x"], arr["x_y"], arr["x_z"])).astype(np.float32)
    x_des = np.column_stack((arr["x_des_x"], arr["x_des_y"], arr["x_des_z"])).astype(np.float32)
    quat_des_xyzw = np.column_stack(
        (arr["quat_des_x"], arr["quat_des_y"], arr["quat_des_z"], arr["quat_des_w"])
    ).astype(np.float32)
    quat_des_wxyz = np.column_stack(
        (quat_des_xyzw[:, 3], quat_des_xyzw[:, 0], quat_des_xyzw[:, 1], quat_des_xyzw[:, 2])
    ).astype(np.float32)

    real = {
        "t_s": torch.from_numpy(t).to(device),
        "q": torch.from_numpy(q).to(device),
        "dq": torch.from_numpy(dq).to(device),
        "x": torch.from_numpy(x).to(device),
        "x_des": torch.from_numpy(x_des).to(device),
        "dx_des": torch.zeros_like(torch.from_numpy(x).to(device)),  # placeholder for interface parity
        "quat_des_wxyz": torch.from_numpy(quat_des_wxyz).to(device),
    }
    return real, sidecar


def format_eta(seconds: float) -> str:
    s = int(max(0, round(seconds)))
    m, s = divmod(s, 60)
    h, m = divmod(m, 60)
    if h > 0:
        return f"{h}h{m:02d}m{s:02d}s"
    if m > 0:
        return f"{m}m{s:02d}s"
    return f"{s}s"


def save_json(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def save_convergence_plot(history: list[dict], out_path: Path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    it = np.array([h["iter"] for h in history], dtype=np.int64)
    best = np.array([h["best"] for h in history], dtype=np.float64)
    mean = np.array([h["mean"] for h in history], dtype=np.float64)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(it, best, label="best", color="C0")
    ax.plot(it, mean, label="mean", color="C1")
    ax.set_xlabel("iteration")
    ax.set_ylabel("fitness")
    ax.set_title("CMA-ES convergence (Franka sysid)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def _consistent_gains(sidecars: list[dict]) -> tuple[float, float]:
    kp_pos = float(sidecars[0]["args"]["kp_pos"])
    kp_ori = float(sidecars[0]["args"]["kp_ori"])
    for i, sc in enumerate(sidecars[1:], start=1):
        kp_pos_i = float(sc["args"]["kp_pos"])
        kp_ori_i = float(sc["args"]["kp_ori"])
        if abs(kp_pos_i - kp_pos) > 1e-6 or abs(kp_ori_i - kp_ori) > 1e-6:
            raise ValueError(
                f"Trajectory {i}: gains (kp_pos={kp_pos_i}, kp_ori={kp_ori_i}) "
                f"differ from traj 0 (kp_pos={kp_pos}, kp_ori={kp_ori}). "
                "Joint sysid currently assumes shared gains."
            )
    return kp_pos, kp_ori


def main():
    args = args_cli
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir).expanduser().resolve() / timestamp
    out_dir.mkdir(parents=True, exist_ok=True)

    real_csvs = [Path(p).expanduser().resolve() for p in args.real_csv]
    real_sidecars = [Path(p).expanduser().resolve() for p in args.real_sidecar]
    if len(real_csvs) != len(real_sidecars):
        raise ValueError(
            f"--real_csv ({len(real_csvs)}) and --real_sidecar ({len(real_sidecars)}) "
            "must come in matching pairs."
        )
    if args.traj_weights is None:
        traj_w = [1.0 for _ in real_csvs]
    else:
        traj_w = [float(v) for v in args.traj_weights.split(",")]
        if len(traj_w) != len(real_csvs):
            raise ValueError("--traj_weights count must match --real_csv count.")

    trajectories = []  # list of dicts with t_s/q/dq/x/x_des/dx_des/quat_des_wxyz/q_init/sidecar/csv
    for csv_path, side_path, weight in zip(real_csvs, real_sidecars, traj_w):
        real, sidecar = load_real_data(csv_path, side_path, args.max_t_s, args.device)
        T_i = int(real["q"].shape[0])
        dt_i = float(real["t_s"][1] - real["t_s"][0]) if T_i > 1 else 0.001
        trajectories.append(
            {
                "csv": csv_path,
                "sidecar_path": side_path,
                "sidecar": sidecar,
                "t_s": real["t_s"],
                "q": real["q"],
                "dq": real["dq"],
                "x": real["x"],
                "x_des": real["x_des"],
                "dx_des": real["dx_des"],
                "quat_des_wxyz": real["quat_des_wxyz"],
                "q_init": [float(v) for v in sidecar["q_init"]],
                "T": T_i,
                "dt": dt_i,
                "weight": float(weight),
            }
        )

    kp_pos, kp_ori = _consistent_gains([t["sidecar"] for t in trajectories])
    t_end_max = max(float(t["t_s"][-1]) for t in trajectories)

    print("\n" + "=" * 72)
    print("Franka System ID - CMA-ES (Task-impedance replay, multi-trajectory)")
    print("=" * 72)
    for i, t in enumerate(trajectories):
        print(f"  traj[{i}] csv     : {t['csv']}")
        print(f"           sidecar : {t['sidecar_path']}")
        print(f"           samples : {t['T']} ({float(t['t_s'][-1]):.3f}s @ dt={t['dt']*1000:.2f}ms) weight={t['weight']}")
    print(f"population    : {args.num_envs}")
    print(f"max_iter      : {args.max_iter}")
    print(f"weights       : w_q={args.w_q}, w_dq={args.w_dq}, w_x={args.w_x}")
    print(f"gains         : kp_pos={kp_pos}, kp_ori={kp_ori}")
    print(f"output_dir    : {out_dir}")

    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.decimation = 1
    env_cfg.sim.dt = 1.0 / 1000.0
    env_cfg.episode_length_s = t_end_max + 1.0  # plenty of headroom for the longest traj
    env_cfg.q_init = trajectories[0]["q_init"]
    env_cfg.control_mode = "task_impedance"
    env_cfg.use_nullspace = False
    env_cfg.traj_npz_path = None
    env_cfg.ctrl.operation_space_cfg.task_prop_gains = [kp_pos, kp_pos, kp_pos, kp_ori, kp_ori, kp_ori]
    env_cfg.ctrl.operation_space_cfg.task_deriv_scale = 1.0

    env = gym.make(args.task, cfg=env_cfg)

    bounds = build_bounds(args)
    cma = CMAES(num_params=NUM_PARAMS, population_size=args.num_envs, sigma=args.sigma, bounds=bounds)

    history = []
    best_score_ever = float("inf")
    best_params_ever = None
    ema_iter_s = None

    try:
        env.reset()
        unwrapped = env.unwrapped
        robot: Articulation = unwrapped.scene["robot"]
        arm_joint_ids = robot.find_joints(ARM_JOINT_NAMES)[0]
        num_joints = robot.num_joints

        for it in range(args.max_iter):
            iter_start = time.time()
            params_np = cma.ask()  # (N,29), real-space params
            params_t = torch.tensor(params_np, dtype=torch.float32, device=unwrapped.device)

            score_total = torch.zeros(args.num_envs, dtype=torch.float32, device=unwrapped.device)
            per_traj_scores = []
            for traj in trajectories:
                # Reset to this trajectory's q_init.  q_init is read at reset time
                # by FrankaTwinSysidEnv._reset_idx, so mutating cfg.q_init here is safe.
                unwrapped.cfg.q_init = traj["q_init"]
                env.reset()
                unwrapped.set_targets(traj["x_des"], traj["dx_des"], traj["quat_des_wxyz"])
                apply_params_to_envs(robot, params_t, arm_joint_ids, num_joints, delay_max=args.delay_max)

                score_q = torch.zeros(args.num_envs, dtype=torch.float32, device=unwrapped.device)
                score_dq = torch.zeros_like(score_q)
                score_x = torch.zeros_like(score_q)

                q_real = traj["q"]
                dq_real = traj["dq"]
                x_real = traj["x"]
                T_i = traj["T"]

                with torch.no_grad():
                    for t_idx in range(T_i):
                        snap = unwrapped.step_replay(t_idx)
                        q_sim = snap["joint_pos"]
                        dq_sim = snap["joint_vel"]
                        x_sim = snap["fingertip_pos"]

                        q_t = q_real[t_idx].unsqueeze(0)
                        dq_t = dq_real[t_idx].unsqueeze(0)
                        x_t = x_real[t_idx].unsqueeze(0)

                        score_q += torch.mean((q_sim - q_t) ** 2, dim=1)
                        score_dq += torch.mean((dq_sim - dq_t) ** 2, dim=1)
                        score_x += torch.mean((x_sim - x_t) ** 2, dim=1)

                score_i = args.w_q * (score_q / T_i) + args.w_dq * (score_dq / T_i) + args.w_x * (score_x / T_i)
                score_total = score_total + traj["weight"] * score_i
                per_traj_scores.append(float(torch.min(score_i).item()))

            score_np = score_total.detach().cpu().numpy()
            cma.tell(score_np)

            min_score = float(np.min(score_np))
            mean_score = float(np.mean(score_np))
            best_idx = int(np.argmin(score_np))
            if min_score < best_score_ever:
                best_score_ever = min_score
                best_params_ever = params_np[best_idx].copy()

            iter_s = time.time() - iter_start
            if ema_iter_s is None:
                ema_iter_s = iter_s
            else:
                ema_iter_s = 0.8 * ema_iter_s + 0.2 * iter_s
            eta_s = ema_iter_s * max(0, args.max_iter - it - 1)

            history.append(
                {
                    "iter": it + 1,
                    "min": min_score,
                    "mean": mean_score,
                    "best": float(best_score_ever),
                    "iter_s": iter_s,
                    "eta_s": eta_s,
                    "per_traj_min": per_traj_scores,
                }
            )
            per_traj_str = " ".join(f"t{i}={s:.2e}" for i, s in enumerate(per_traj_scores))
            print(
                f"[{it+1:3d}/{args.max_iter:3d}] "
                f"best={best_score_ever:.6e} mean={mean_score:.6e} {per_traj_str} "
                f"iter={iter_s:5.1f}s ETA={format_eta(eta_s)}"
            )

            if (it + 1) % args.save_interval == 0:
                ckpt = {
                    "iter": it + 1,
                    "best_score": best_score_ever,
                    "best_params": best_params_ever.tolist() if best_params_ever is not None else None,
                    "history": history,
                    "bounds": bounds.tolist(),
                    "args": vars(args),
                }
                if best_params_ever is not None:
                    dec = decode_params(best_params_ever)
                    ckpt["best_params_decoded"] = {
                        "armature": dec["armature"].tolist(),
                        "mu_static": dec["mu_static"].tolist(),
                        "dynamic_ratio": dec["dynamic_ratio"].tolist(),
                        "mu_dynamic": dec["mu_dynamic"].tolist(),
                        "mu_viscous": dec["mu_viscous"].tolist(),
                        "motor_delay_steps": int(dec["motor_delay_steps"]),
                    }
                save_json(out_dir / f"checkpoint_{it+1:04d}.json", ckpt)

        total_s = sum(h["iter_s"] for h in history)
        print(f"\nDONE in {format_eta(total_s)}. best_score={best_score_ever:.6e}")

        if best_params_ever is None:
            raise RuntimeError("CMA-ES finished without best params.")
        decoded = decode_params(best_params_ever)
        result_payload = {
            "best_score": float(best_score_ever),
            "best_params_raw": best_params_ever.tolist(),
            "best_params_decoded": {
                "armature": decoded["armature"].tolist(),
                "mu_static": decoded["mu_static"].tolist(),
                "dynamic_ratio": decoded["dynamic_ratio"].tolist(),
                "mu_dynamic": decoded["mu_dynamic"].tolist(),
                "mu_viscous": decoded["mu_viscous"].tolist(),
                "motor_delay_steps": int(decoded["motor_delay_steps"]),
            },
            "history": history,
            "bounds": bounds.tolist(),
            "args": vars(args),
        }
        result_payload["source"] = {
            "trajectories": [
                {
                    "real_csv": str(t["csv"]),
                    "real_sidecar": str(t["sidecar_path"]),
                    "num_samples": int(t["T"]),
                    "weight": float(t["weight"]),
                }
                for t in trajectories
            ],
            "num_trajectories": len(trajectories),
        }
        save_json(out_dir / "sysid_best_params.json", result_payload)
        save_convergence_plot(history, out_dir / "convergence.png")
        print(f"Saved: {out_dir / 'sysid_best_params.json'}")
        print(f"Saved: {out_dir / 'convergence.png'}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
