# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Franka system identification via CMA-ES (task-impedance closed-loop replay).

Supports **multi-trajectory** joint fitting: pass ``--real_csv`` and
``--real_sidecar`` multiple times to optimise over a sum of per-trajectory
losses.  A single slow trajectory (e.g. a 0.25 Hz z-sine) leaves armature /
viscous / delay under-identified; adding a multi-band sweep or a chirp supplies
the 0.7-1.1 Hz acceleration content needed to pin them down.

Time base matches ``replay_python_csv_sim.py`` exactly: after a warmup at the
initial pose, CSV row ``k`` is sampled and its target applied, then the sim is
stepped ``round((t[k+1] - t[k]) / 1 ms)`` physics ticks with the target held
(zero-order hold).  The loss is evaluated only at the CSV sample instants.

Trajectories run in parallel env blocks: with population ``P`` (``--num_envs``)
and ``B`` trajectories the sim holds ``P * B`` envs, and env ``b * P + i``
replays trajectory ``b`` with CMA candidate ``i`` (using that trajectory's own
q_init and controller gains).
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Franka system ID (CMA-ES) by closed-loop replay of real trajectories.")
parser.add_argument("--task", type=str, default="Isaac-FrankaTwin-Sysid-v0")
parser.add_argument(
    "--num_envs",
    type=int,
    default=128,
    help="CMA population size. Total sim envs = num_envs x number of trajectories.",
)
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
parser.add_argument("--warmup_steps", type=int, default=50, help="1 kHz warmup ticks at the initial pose (same as replay).")
parser.add_argument(
    "--eval_params",
    type=str,
    default=None,
    help="Skip CMA-ES: roll out this fixed params JSON (sysid_best_params.json format) and save the sim rollouts.",
)
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
SIM_DT = 1.0 / 1000.0


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


def decoded_to_json(params: np.ndarray) -> dict:
    dec = decode_params(params)
    return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in dec.items()}


def load_params_json(path: Path) -> np.ndarray:
    """Inverse of ``decode_params`` for a sysid_best_params.json / checkpoint-style file."""
    payload = json.loads(Path(path).read_text())
    p = payload.get("best_params_decoded", payload)
    vec = list(p["armature"]) + list(p["mu_static"]) + list(p["dynamic_ratio"]) + list(p["mu_viscous"])
    vec.append(float(p["motor_delay_steps"]))
    return np.asarray(vec, dtype=np.float64)


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


def compute_step_counts(t_csv: np.ndarray, sim_dt: float) -> np.ndarray:
    """1 kHz ticks to hold target[k] before sampling row k+1 (same as replay_python_csv_sim)."""
    n = len(t_csv)
    steps = np.ones(n, dtype=np.int64)
    if n >= 2:
        steps[: n - 1] = np.clip(np.round(np.diff(t_csv) / sim_dt).astype(np.int64), 1, None)
    return steps


def load_real_data(csv_path: Path, sidecar_path: Path, max_t_s: float | None, device: str):
    with open(sidecar_path, "r", encoding="utf-8") as f:
        sidecar = json.load(f)
    arr = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=np.float64)
    arr = np.atleast_1d(arr)
    if arr.size == 0:
        raise ValueError(f"Empty real csv: {csv_path}")
    if max_t_s is not None:
        arr = arr[arr["t_s"] <= float(max_t_s)]

    t = np.asarray(arr["t_s"], dtype=np.float64)
    q = np.column_stack([arr[f"q{i}"] for i in range(1, 8)])
    dq = np.column_stack([arr[f"dq{i}"] for i in range(1, 8)])
    x = np.column_stack((arr["x_x"], arr["x_y"], arr["x_z"]))
    x_des = np.column_stack((arr["x_des_x"], arr["x_des_y"], arr["x_des_z"]))
    quat_des_wxyz = np.column_stack((arr["quat_des_w"], arr["quat_des_x"], arr["quat_des_y"], arr["quat_des_z"]))
    quat_des_wxyz /= np.clip(np.linalg.norm(quat_des_wxyz, axis=1, keepdims=True), 1e-12, None)

    def to_t(a):
        return torch.as_tensor(a, dtype=torch.float32, device=device)

    real = {
        "t_s": t,
        "q": to_t(q),
        "dq": to_t(dq),
        "x": to_t(x),
        "x_des": to_t(x_des),
        "quat_des_wxyz": to_t(quat_des_wxyz),
        "step_counts": compute_step_counts(t, SIM_DT),
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


def _sidecar_gains(sidecar: dict) -> dict:
    """Task-space PD gains the real controller ran with (kd defaults to critical damping)."""
    a = sidecar["args"]
    kp_pos = float(a["kp_pos"])
    kp_ori = float(a["kp_ori"])
    kd_pos = float(a["kd_pos"]) if a.get("kd_pos") is not None else 2.0 * float(np.sqrt(kp_pos))
    kd_ori = float(a["kd_ori"]) if a.get("kd_ori") is not None else 2.0 * float(np.sqrt(kp_ori))
    return {"kp_pos": kp_pos, "kp_ori": kp_ori, "kd_pos": kd_pos, "kd_ori": kd_ori}


def set_block_gains(unwrapped, env_slice: slice, gains: dict):
    """Overwrite one env block's task gains in place; read every control step by the env."""
    kp = torch.tensor([gains["kp_pos"]] * 3 + [gains["kp_ori"]] * 3, device=unwrapped.device)
    kd = torch.tensor([gains["kd_pos"]] * 3 + [gains["kd_ori"]] * 3, device=unwrapped.device)
    unwrapped.task_prop_gains[env_slice] = kp
    unwrapped.task_deriv_gains[env_slice] = kd


def build_schedule(trajectories: list[dict]) -> tuple[dict[int, list[tuple[int, int]]], int]:
    """Map post-warmup tick -> [(traj b, csv row k)] to sample and re-target at that tick."""
    events: dict[int, list[tuple[int, int]]] = {}
    total_ticks = 0
    for b, traj in enumerate(trajectories):
        steps = traj["step_counts"]
        starts = np.concatenate(([0], np.cumsum(steps[:-1])))
        for k, tick in enumerate(starts.tolist()):
            events.setdefault(int(tick), []).append((b, k))
        total_ticks = max(total_ticks, int(steps.sum()))
    return events, total_ticks


def rollout(env, robot, arm_joint_ids, trajectories, pop: int, params_all: torch.Tensor, args, schedule, record=False):
    """Replay every trajectory in its env block with per-env params; return per-env scores (B, pop).

    With ``record``, also returns the sampled (q, dq, x) of each block's first env.
    """
    unwrapped = env.unwrapped
    device = unwrapped.device
    events, total_ticks = schedule
    n_traj = len(trajectories)
    blocks = [slice(b * pop, (b + 1) * pop) for b in range(n_traj)]

    env.reset()  # every block resets to its own q_init; ctrl targets = current fingertip pose
    apply_params_to_envs(robot, params_all, arm_joint_ids, robot.num_joints, delay_max=args.delay_max)

    score_q = torch.zeros(unwrapped.num_envs, device=device)
    score_dq = torch.zeros_like(score_q)
    score_x = torch.zeros_like(score_q)
    recs = [{"q": [], "dq": [], "x": []} for _ in range(n_traj)] if record else None
    tgt_pos = unwrapped.ctrl_target_fingertip_midpoint_pos
    tgt_quat = unwrapped.ctrl_target_fingertip_midpoint_quat

    # no_grad, not inference_mode: sim buffers touched here are written in place by the next env.reset().
    with torch.no_grad():
        for _ in range(args.warmup_steps):
            unwrapped.physics_step()
        for tick in range(total_ticks):
            for b, k in events.get(tick, ()):
                sl = blocks[b]
                traj = trajectories[b]
                q_sim, dq_sim, x_sim = unwrapped.arm_state()
                score_q[sl] += torch.mean((q_sim[sl] - traj["q"][k]) ** 2, dim=1)
                score_dq[sl] += torch.mean((dq_sim[sl] - traj["dq"][k]) ** 2, dim=1)
                score_x[sl] += torch.mean((x_sim[sl] - traj["x"][k]) ** 2, dim=1)
                if record:
                    recs[b]["q"].append(q_sim[sl.start].clone())
                    recs[b]["dq"].append(dq_sim[sl.start].clone())
                    recs[b]["x"].append(x_sim[sl.start].clone())
                tgt_pos[sl] = traj["x_des"][k]
                tgt_quat[sl] = traj["quat_des_wxyz"][k]
            unwrapped.physics_step()

    per_block = []
    for b, traj in enumerate(trajectories):
        sl = blocks[b]
        T_b = traj["T"]
        per_block.append(
            args.w_q * score_q[sl] / T_b + args.w_dq * score_dq[sl] / T_b + args.w_x * score_x[sl] / T_b
        )
    scores = torch.stack(per_block)  # (B, pop)
    if record:
        recs = [{k: torch.stack(v).cpu().numpy() for k, v in r.items()} for r in recs]
    return scores, recs


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

    trajectories = []
    for csv_path, side_path, weight in zip(real_csvs, real_sidecars, traj_w):
        real, sidecar = load_real_data(csv_path, side_path, args.max_t_s, args.device)
        trajectories.append(
            {
                **real,
                "csv": csv_path,
                "sidecar_path": side_path,
                "q_init": [float(v) for v in sidecar["q_init"]],
                "gains": _sidecar_gains(sidecar),
                "T": int(real["q"].shape[0]),
                "weight": float(weight),
            }
        )

    pop = int(args.num_envs)
    n_traj = len(trajectories)
    num_envs_total = pop * n_traj
    schedule = build_schedule(trajectories)
    ticks_per_rollout = args.warmup_steps + schedule[1]

    print("\n" + "=" * 72)
    print("Franka System ID - CMA-ES (Task-impedance ZOH replay, parallel trajectory blocks)")
    print("=" * 72)
    for i, t in enumerate(trajectories):
        g = t["gains"]
        print(f"  traj[{i}] csv     : {t['csv']}")
        print(f"           sidecar : {t['sidecar_path']}")
        print(
            f"           samples : {t['T']} ({t['t_s'][-1]:.3f}s, {int(t['step_counts'].sum())} ticks, "
            f"{t['step_counts'].mean():.1f} ticks/row) weight={t['weight']}"
        )
        print(
            f"           gains   : kp_pos={g['kp_pos']}, kp_ori={g['kp_ori']}, "
            f"kd_pos={g['kd_pos']:.3f}, kd_ori={g['kd_ori']:.3f}"
        )
        print(f"           envs    : [{i * pop}, {(i + 1) * pop})")
    print(f"population    : {pop}  (total envs = {num_envs_total})")
    print(f"ticks/rollout : {ticks_per_rollout} (warmup {args.warmup_steps})")
    print(f"max_iter      : {args.max_iter}")
    print(f"weights       : w_q={args.w_q}, w_dq={args.w_dq}, w_x={args.w_x}")
    print(f"output_dir    : {out_dir}", flush=True)

    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=num_envs_total)
    env_cfg.scene.num_envs = num_envs_total
    env_cfg.decimation = 1
    env_cfg.sim.dt = SIM_DT
    env_cfg.episode_length_s = ticks_per_rollout * SIM_DT + 1.0  # unused by physics_step(); kept for safety
    env_cfg.q_init = trajectories[0]["q_init"]  # fallback only; per-env q_init is set below
    env_cfg.control_mode = "task_impedance"
    env_cfg.use_nullspace = False
    env_cfg.traj_npz_path = None
    g0 = trajectories[0]["gains"]
    env_cfg.ctrl.operation_space_cfg.task_prop_gains = [g0["kp_pos"]] * 3 + [g0["kp_ori"]] * 3
    env_cfg.ctrl.operation_space_cfg.task_deriv_scale = 1.0

    env = gym.make(args.task, cfg=env_cfg)

    try:
        unwrapped = env.unwrapped
        device = unwrapped.device
        robot: Articulation = unwrapped.scene["robot"]
        arm_joint_ids = robot.find_joints(ARM_JOINT_NAMES)[0]

        q_init_all = torch.tensor(
            [t["q_init"] for t in trajectories], dtype=torch.float32, device=device
        ).repeat_interleave(pop, dim=0)
        unwrapped.set_q_init_per_env(q_init_all)
        for b, t in enumerate(trajectories):
            set_block_gains(unwrapped, slice(b * pop, (b + 1) * pop), t["gains"])
        weights_t = torch.tensor([t["weight"] for t in trajectories], device=device).unsqueeze(1)

        if args.eval_params is not None:
            params_np = load_params_json(Path(args.eval_params))
            params_all = torch.tensor(params_np, dtype=torch.float32, device=device).repeat(num_envs_total, 1)
            t0 = time.time()
            scores, recs = rollout(env, robot, arm_joint_ids, trajectories, pop, params_all, args, schedule, record=True)
            dt_s = time.time() - t0
            per_traj = scores[:, 0].tolist()
            total = float((weights_t * scores).sum(dim=0)[0])
            print(f"\n[eval] params={args.eval_params}")
            print(f"[eval] rollout {dt_s:.1f}s ({dt_s / ticks_per_rollout * 1e3:.2f} ms/tick)")
            print(f"[eval] score total={total:.6e} per_traj=" + " ".join(f"t{i}={s:.3e}" for i, s in enumerate(per_traj)))
            for b, (t, r) in enumerate(zip(trajectories, recs)):
                cols = ["t_s"] + [f"q{i}" for i in range(1, 8)] + [f"dq{i}" for i in range(1, 8)] + ["x_x", "x_y", "x_z"]
                path = out_dir / f"eval_rollout_{t['csv'].stem}.csv"
                np.savetxt(path, np.column_stack((t["t_s"], r["q"], r["dq"], r["x"])), delimiter=",",
                           header=",".join(cols), comments="", fmt="%.9f")
                print(f"[eval] wrote {path}")
            save_json(out_dir / "eval_result.json", {
                "params_path": str(Path(args.eval_params).resolve()),
                "params_decoded": decoded_to_json(params_np),
                "score_total": total,
                "score_per_traj": per_traj,
                "rollout_s": dt_s,
                "ms_per_tick": dt_s / ticks_per_rollout * 1e3,
                "trajectories": [str(t["csv"]) for t in trajectories],
                "args": vars(args),
            })
            sys.stdout.flush()
            return

        bounds = build_bounds(args)
        cma = CMAES(num_params=NUM_PARAMS, population_size=pop, sigma=args.sigma, bounds=bounds)

        history = []
        best_score_ever = float("inf")
        best_params_ever = None
        ema_iter_s = None

        for it in range(args.max_iter):
            iter_start = time.time()
            params_np = cma.ask()  # (pop, 29), real-space params
            params_t = torch.tensor(params_np, dtype=torch.float32, device=device)
            params_all = params_t.repeat(n_traj, 1)  # env b*pop+i <- candidate i

            scores, _ = rollout(env, robot, arm_joint_ids, trajectories, pop, params_all, args, schedule)
            score_np = (weights_t * scores).sum(dim=0).cpu().numpy()
            per_traj_scores = scores.min(dim=1).values.tolist()
            cma.tell(score_np)

            min_score = float(np.min(score_np))
            mean_score = float(np.mean(score_np))
            best_idx = int(np.argmin(score_np))
            if min_score < best_score_ever:
                best_score_ever = min_score
                best_params_ever = params_np[best_idx].copy()

            iter_s = time.time() - iter_start
            ema_iter_s = iter_s if ema_iter_s is None else 0.8 * ema_iter_s + 0.2 * iter_s
            eta_s = ema_iter_s * max(0, args.max_iter - it - 1)

            history.append(
                {
                    "iter": it + 1,
                    "min": min_score,
                    "mean": mean_score,
                    "best": float(best_score_ever),
                    "iter_s": iter_s,
                    "ms_per_tick": iter_s / ticks_per_rollout * 1e3,
                    "eta_s": eta_s,
                    "per_traj_min": per_traj_scores,
                }
            )
            per_traj_str = " ".join(f"t{i}={s:.2e}" for i, s in enumerate(per_traj_scores))
            print(
                f"[{it+1:3d}/{args.max_iter:3d}] "
                f"best={best_score_ever:.6e} mean={mean_score:.6e} {per_traj_str} "
                f"iter={iter_s:5.1f}s ({iter_s / ticks_per_rollout * 1e3:.2f} ms/tick) ETA={format_eta(eta_s)}",
                flush=True,
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
                    ckpt["best_params_decoded"] = decoded_to_json(best_params_ever)
                save_json(out_dir / f"checkpoint_{it+1:04d}.json", ckpt)

        total_s = sum(h["iter_s"] for h in history)
        print(f"\nDONE in {format_eta(total_s)}. best_score={best_score_ever:.6e}")

        if best_params_ever is None:
            raise RuntimeError("CMA-ES finished without best params.")
        result_payload = {
            "best_score": float(best_score_ever),
            "best_params_raw": best_params_ever.tolist(),
            "best_params_decoded": decoded_to_json(best_params_ever),
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
                    "gains": t["gains"],
                }
                for t in trajectories
            ],
            "num_trajectories": n_traj,
            "time_base": "zoh_1khz_ticks_per_csv_row",
            "warmup_steps": int(args.warmup_steps),
        }
        save_json(out_dir / "sysid_best_params.json", result_payload)
        save_convergence_plot(history, out_dir / "convergence.png")
        print(f"Saved: {out_dir / 'sysid_best_params.json'}")
        print(f"Saved: {out_dir / 'convergence.png'}", flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
