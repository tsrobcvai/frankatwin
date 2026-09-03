# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Replay a Python-driven (low-rate) Cartesian trajectory in IsaacLab.

Consumes the lower-rate CSV produced by ``examples/cart_impedance.py``
(typically a 50 Hz Python set_ee_target loop) and reproduces the same
zero-order-hold (ZOH) target staircase that the real robot saw.

Per-iteration flow (matches the Python real-side loop in cart_impedance.py):

  1. Sample sim state and write the output CSV row.
  2. Apply the new target ``(x_des[k], quat_des[k])``.
  3. Step the sim ``round((t_csv[k+1] - t_csv[k]) * 1000)`` 1 ms physics
     ticks. The target is held constant for that whole window — this is
     where we mirror what the real osc_shm saw.

So row ``k`` in the sim CSV holds state at time ``t_csv[k]`` under target
``[k-1]`` (or the anchor warmup target for k=0). Same time alignment as
``examples/cart_impedance.py`` produces on the real side, so comparison
needs no interpolation.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="Replay a Python-driven low-rate Cartesian trajectory in IsaacLab with ZOH."
)
parser.add_argument("--task", type=str, default="Isaac-FrankaTwin-Replay-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--real-csv", type=str, required=True,
                    help="Path to the Python-side CSV (from cart_impedance.py --log).")
parser.add_argument("--real-sidecar", type=str, required=True,
                    help="Path to the matching sidecar JSON.")
parser.add_argument("--out-csv", type=str, default=None,
                    help="Output sim CSV path (default: <real>_sim.csv).")
parser.add_argument("--out-sidecar", type=str, default=None,
                    help="Output sim sidecar JSON path (default: <real>_sim.json).")
parser.add_argument("--gain-source", type=str, default="sidecar",
                    choices=["sidecar", "env_cfg"],
                    help="Use gains from real sidecar or Isaac-UW task defaults.")
parser.add_argument("--warmup-steps", type=int, default=50,
                    help="Number of 1 kHz warmup steps at the initial target.")
parser.add_argument("--use-nullspace", action="store_true",
                    help="Enable OSC nullspace projection (default OFF, matches osc_shm).")
parser.add_argument("--control-mode", type=str, default="task_impedance",
                    choices=["task_impedance", "osc"],
                    help="Cartesian controller mode (default 'task_impedance' matches osc_shm).")
parser.add_argument("--sysid-params", type=str, default=None,
                    help="Optional sysid_best_params.json to patch arm dynamics/delay.")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

# Resolve user-supplied input paths to absolute BEFORE we chdir into the
# IsaacLab repo root (some robot configs use relative "./source/..." USD paths
# that only resolve correctly when CWD == IsaacLab root).
for _path_attr in ("real_csv", "real_sidecar", "out_csv", "out_sidecar", "sysid_params"):
    _val = getattr(args_cli, _path_attr, None)
    if _val is not None:
        setattr(args_cli, _path_attr, str(Path(_val).expanduser().resolve()))

_ISAACLAB_ROOT = Path(__file__).resolve().parent.parent.parent
os.chdir(_ISAACLAB_ROOT)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from isaaclab.actuators.actuator_cfg import DelayedPDActuatorCfg  # noqa: E402
import isaaclab_tasks  # noqa: F401, E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


REQUIRED_TARGET_COLUMNS = [
    "t_s",
    "x_des_x", "x_des_y", "x_des_z",
    "dx_des_x", "dx_des_y", "dx_des_z",
    "quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w",
]


def xyzw_to_wxyz(quat_xyzw: np.ndarray) -> np.ndarray:
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=np.float64)


def wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64)


def quat_normalize_xyzw(quat_xyzw: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quat_xyzw, axis=-1, keepdims=True)
    norm = np.clip(norm, 1e-12, None)
    return quat_xyzw / norm


def quat_conjugate_xyzw(quat_xyzw: np.ndarray) -> np.ndarray:
    out = quat_xyzw.copy()
    out[..., :3] *= -1.0
    return out


def quat_multiply_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    w = aw * bw - ax * bx - ay * by - az * bz
    return np.stack((x, y, z, w), axis=-1)


def orientation_error_vec_xyzw(quat_des_xyzw: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    quat_des_xyzw = quat_normalize_xyzw(quat_des_xyzw)
    quat_xyzw = quat_normalize_xyzw(quat_xyzw)
    q_err = quat_multiply_xyzw(quat_des_xyzw, quat_conjugate_xyzw(quat_xyzw))
    flip_mask = q_err[:, 3] < 0.0
    q_err[flip_mask] *= -1.0
    return 2.0 * q_err[:, :3]


def quat_shortest_angle_xyzw(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> float:
    q1_xyzw = quat_normalize_xyzw(q1_xyzw.reshape(1, 4))[0]
    q2_xyzw = quat_normalize_xyzw(q2_xyzw.reshape(1, 4))[0]
    dot_val = float(np.clip(abs(np.dot(q1_xyzw, q2_xyzw)), 0.0, 1.0))
    return float(2.0 * math.acos(dot_val))


def resolve_output_paths(real_csv_path: Path, out_csv: str | None, out_sidecar: str | None) -> tuple[Path, Path]:
    out_csv_path = (
        Path(out_csv) if out_csv is not None else real_csv_path.with_name(f"{real_csv_path.stem}_sim.csv")
    )
    out_sidecar_path = (
        Path(out_sidecar) if out_sidecar is not None
        else real_csv_path.with_name(f"{real_csv_path.stem}_sim.json")
    )
    return out_csv_path.resolve(), out_sidecar_path.resolve()


def compute_summary(e_pos: np.ndarray, e_ori: np.ndarray, t_s: np.ndarray) -> dict:
    return {
        "ticks": int(len(t_s)),
        "elapsed_s": float(t_s[-1]) if len(t_s) > 0 else 0.0,
        "err_pos_rms_xyz": np.sqrt(np.mean(e_pos**2, axis=0)).tolist(),
        "err_pos_abs_max_xyz": np.max(np.abs(e_pos), axis=0).tolist(),
        "err_pos_inf_max": float(np.max(np.linalg.norm(e_pos, axis=1))),
        "err_ori_rms_xyz": np.sqrt(np.mean(e_ori**2, axis=0)).tolist(),
        "err_ori_abs_max_xyz": np.max(np.abs(e_ori), axis=0).tolist(),
        "err_ori_norm_max": float(np.max(np.linalg.norm(e_ori, axis=1))),
    }


def load_real_csv(path: Path):
    real = np.genfromtxt(path, delimiter=",", names=True, dtype=np.float64)
    real = np.atleast_1d(real)
    if real.size == 0:
        raise ValueError(f"Real csv is empty: {path}")
    for col in REQUIRED_TARGET_COLUMNS:
        if col not in real.dtype.names:
            raise KeyError(f"Missing required csv column '{col}' in {path}")
    return real


def _load_sysid_params(path: str | None) -> dict | None:
    if path is None:
        return None
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    params = payload.get("best_params_decoded", payload)
    required = ["armature", "mu_static", "dynamic_ratio", "mu_viscous", "motor_delay_steps"]
    missing = [k for k in required if k not in params]
    if missing:
        raise KeyError(f"sysid params file missing keys: {missing}")
    if len(params["armature"]) != 7:
        raise ValueError("sysid armature must have length 7")
    if len(params["mu_static"]) != 7 or len(params["dynamic_ratio"]) != 7 or len(params["mu_viscous"]) != 7:
        raise ValueError("sysid friction arrays must have length 7")
    return {
        "armature": [float(v) for v in params["armature"]],
        "mu_static": [float(v) for v in params["mu_static"]],
        "dynamic_ratio": [float(v) for v in params["dynamic_ratio"]],
        "mu_dynamic": [float(v) * float(r) for v, r in zip(params["mu_static"], params["dynamic_ratio"])],
        "mu_viscous": [float(v) for v in params["mu_viscous"]],
        "motor_delay_steps": int(params["motor_delay_steps"]),
        "source_path": str(path),
    }


def _enable_delayed_arm_actuators(env_cfg, delay_max: int):
    env_cfg.robot.actuators["panda_arm1"] = DelayedPDActuatorCfg(
        joint_names_expr=["panda_joint[1-4]"],
        stiffness=0.0, damping=0.0, friction=0.0, armature=0.0,
        effort_limit=100.0, velocity_limit=124.6,
        effort_limit_sim=100.0, velocity_limit_sim=124.6,
        min_delay=0, max_delay=max(1, int(delay_max)),
    )
    env_cfg.robot.actuators["panda_arm2"] = DelayedPDActuatorCfg(
        joint_names_expr=["panda_joint[5-7]"],
        stiffness=0.0, damping=0.0, friction=0.0, armature=0.0,
        effort_limit=50.0, velocity_limit=149.5,
        effort_limit_sim=50.0, velocity_limit_sim=149.5,
        min_delay=0, max_delay=max(1, int(delay_max)),
    )


def _apply_sysid_params_to_robot(robot, sysid_params: dict, num_envs: int, device: torch.device):
    arm_joint_ids = robot.find_joints([f"panda_joint{i}" for i in range(1, 8)])[0]
    num_joints = robot.num_joints
    env_ids = torch.arange(num_envs, device=device)

    armature_full = torch.zeros((num_envs, num_joints), device=device)
    static_fric_full = torch.zeros((num_envs, num_joints), device=device)
    dynamic_fric_full = torch.zeros((num_envs, num_joints), device=device)
    viscous_fric_full = torch.zeros((num_envs, num_joints), device=device)

    armature = torch.tensor(sysid_params["armature"], dtype=torch.float32, device=device).unsqueeze(0).repeat(num_envs, 1)
    mu_static = torch.tensor(sysid_params["mu_static"], dtype=torch.float32, device=device).unsqueeze(0).repeat(num_envs, 1)
    mu_dynamic = torch.tensor(sysid_params["mu_dynamic"], dtype=torch.float32, device=device).unsqueeze(0).repeat(num_envs, 1)
    mu_viscous = torch.tensor(sysid_params["mu_viscous"], dtype=torch.float32, device=device).unsqueeze(0).repeat(num_envs, 1)

    armature_full[:, arm_joint_ids] = armature
    static_fric_full[:, arm_joint_ids] = mu_static
    dynamic_fric_full[:, arm_joint_ids] = mu_dynamic
    viscous_fric_full[:, arm_joint_ids] = mu_viscous

    robot.write_joint_armature_to_sim(armature_full, env_ids=env_ids)
    robot.write_joint_friction_coefficient_to_sim(
        static_fric_full,
        joint_dynamic_friction_coeff=dynamic_fric_full,
        joint_viscous_friction_coeff=viscous_fric_full,
        env_ids=env_ids,
    )

    delay_int = torch.full((num_envs,), int(sysid_params["motor_delay_steps"]), dtype=torch.int, device=device)
    for act_name in ("panda_arm1", "panda_arm2"):
        actuator = robot.actuators.get(act_name)
        if actuator is None:
            continue
        if hasattr(actuator, "positions_delay_buffer"):
            actuator.positions_delay_buffer.set_time_lag(delay_int, env_ids)
            actuator.velocities_delay_buffer.set_time_lag(delay_int, env_ids)
            actuator.efforts_delay_buffer.set_time_lag(delay_int, env_ids)


def _compute_step_counts(t_csv: np.ndarray, sim_dt: float) -> np.ndarray:
    """Return number of 1 kHz physics steps to advance between consecutive CSV rows.

    For each CSV row k, steps[k] = how many sim-dt ticks to step *after* applying
    target[k] before recording row k+1. The last row gets a trailing 1-tick window
    so the final state isn't sampled mid-transient.
    """
    n = len(t_csv)
    steps = np.ones(n, dtype=np.int64)
    if n >= 2:
        diffs_ms = np.diff(t_csv) / sim_dt
        steps[: n - 1] = np.clip(np.round(diffs_ms).astype(np.int64), 1, None)
    return steps


def main():
    real_csv_path = Path(args_cli.real_csv).expanduser().resolve()
    real_sidecar_path = Path(args_cli.real_sidecar).expanduser().resolve()
    out_csv_path, out_sidecar_path = resolve_output_paths(real_csv_path, args_cli.out_csv, args_cli.out_sidecar)

    with open(real_sidecar_path, "r", encoding="utf-8") as f:
        real_sidecar = json.load(f)
    real = load_real_csv(real_csv_path)

    q_init = [float(v) for v in real_sidecar["q_init"]]
    x_anchor = np.asarray(real_sidecar["x_anchor"], dtype=np.float64)
    q_anchor_xyzw = np.asarray(real_sidecar["q_anchor_xyzw"], dtype=np.float64)
    q_anchor_xyzw = quat_normalize_xyzw(q_anchor_xyzw.reshape(1, 4))[0]

    real_args = real_sidecar.get("args", {})
    if args_cli.gain_source == "sidecar":
        kp_pos = float(real_args["kp_pos"])
        kp_ori = float(real_args["kp_ori"])
    else:
        kp_pos = 75.0
        kp_ori = 150.0

    t_s = np.asarray(real["t_s"], dtype=np.float64)
    x_des = np.column_stack((real["x_des_x"], real["x_des_y"], real["x_des_z"]))
    dx_des = np.column_stack((real["dx_des_x"], real["dx_des_y"], real["dx_des_z"]))
    quat_des_xyzw = np.column_stack(
        (real["quat_des_x"], real["quat_des_y"], real["quat_des_z"], real["quat_des_w"])
    )
    quat_des_xyzw = quat_normalize_xyzw(quat_des_xyzw)

    sysid_params = _load_sysid_params(args_cli.sysid_params)

    sim_dt = 1.0 / 1000.0
    step_counts = _compute_step_counts(t_s, sim_dt)
    total_phys_steps = int(step_counts.sum())
    duration_s = float(t_s[-1]) if len(t_s) > 0 else 0.0
    episode_length_s = max(duration_s + 0.5, total_phys_steps * sim_dt + 0.1)

    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.scene.num_envs = 1
    env_cfg.decimation = 1
    env_cfg.sim.dt = sim_dt
    env_cfg.episode_length_s = episode_length_s
    env_cfg.q_init = q_init
    env_cfg.ctrl.operation_space_cfg.task_prop_gains = [kp_pos, kp_pos, kp_pos, kp_ori, kp_ori, kp_ori]
    env_cfg.ctrl.operation_space_cfg.task_deriv_scale = 1.0
    env_cfg.use_nullspace = bool(args_cli.use_nullspace)
    env_cfg.control_mode = str(args_cli.control_mode)
    if sysid_params is not None:
        _enable_delayed_arm_actuators(env_cfg, delay_max=max(4, int(sysid_params["motor_delay_steps"])))

    print(f"[replay_python_csv_sim] gym.make('{args_cli.task}')")
    print(f"[replay_python_csv_sim] csv rows = {len(t_s)}, total phys steps = {total_phys_steps} "
          f"(~{total_phys_steps * sim_dt:.2f} s sim time)")
    print(f"[replay_python_csv_sim] mean steps/csv-row = {step_counts.mean():.1f} "
          f"(=> effective control rate {1.0 / (step_counts.mean() * sim_dt):.1f} Hz)")
    env = gym.make(args_cli.task, cfg=env_cfg)

    try:
        env.reset()
        unwrapped = env.unwrapped
        device = torch.device(unwrapped.device)
        if sysid_params is not None:
            _apply_sysid_params_to_robot(unwrapped._robot, sysid_params, unwrapped.num_envs, device)
            print(f"[replay_python_csv_sim] applied sysid params: "
                  f"delay={sysid_params['motor_delay_steps']} source={sysid_params['source_path']}")

        snapshot_init = unwrapped.get_state_snapshot()
        init_pos = np.asarray(snapshot_init["x"], dtype=np.float64)
        init_quat_xyzw = wxyz_to_xyzw(np.asarray(snapshot_init["quat_wxyz"], dtype=np.float64))

        pos_anchor_err = float(np.linalg.norm(init_pos - x_anchor))
        ori_anchor_err = quat_shortest_angle_xyzw(init_quat_xyzw, q_anchor_xyzw)
        if pos_anchor_err > 1.0e-3 or ori_anchor_err > 1.0e-2:
            raise RuntimeError(
                "FK sanity check failed at q_init: "
                f"||x_sim-x_anchor||={pos_anchor_err:.6e} m, "
                f"ori_err={ori_anchor_err:.6e} rad."
            )

        init_pos_t = torch.as_tensor(init_pos, dtype=torch.float32, device=device).unsqueeze(0)
        init_quat_wxyz_t = torch.as_tensor(snapshot_init["quat_wxyz"], dtype=torch.float32, device=device).unsqueeze(0)
        unwrapped.set_target_pose(init_pos_t, init_quat_wxyz_t)

        action_dim = int(getattr(unwrapped.cfg, "action_space", 0) or 0)
        zero_action = torch.zeros((unwrapped.num_envs, action_dim), dtype=torch.float32, device=device)
        for _ in range(max(0, int(args_cli.warmup_steps))):
            env.step(zero_action)

        num_steps = len(t_s)
        q = np.zeros((num_steps, 7), dtype=np.float64)
        dq = np.zeros((num_steps, 7), dtype=np.float64)
        x = np.zeros((num_steps, 3), dtype=np.float64)
        dx = np.zeros((num_steps, 3), dtype=np.float64)
        quat_xyzw = np.zeros((num_steps, 4), dtype=np.float64)
        w = np.zeros((num_steps, 3), dtype=np.float64)

        with torch.inference_mode():
            for k in range(num_steps):
                # 1. Snapshot first — state at t_csv[k] under previous target.
                snap = unwrapped.get_state_snapshot()
                q[k] = snap["q"]
                dq[k] = snap["dq"]
                x[k] = snap["x"]
                dx[k] = snap["dx"]
                quat_xyzw[k] = wxyz_to_xyzw(np.asarray(snap["quat_wxyz"], dtype=np.float64))
                w[k] = snap["w"]

                # 2. Apply target[k].
                target_pos_t = torch.as_tensor(x_des[k], dtype=torch.float32, device=device).unsqueeze(0)
                target_quat_wxyz_t = torch.as_tensor(
                    xyzw_to_wxyz(quat_des_xyzw[k]), dtype=torch.float32, device=device
                ).unsqueeze(0)
                unwrapped.set_target_pose(target_pos_t, target_quat_wxyz_t)

                # 3. Step physics for the ZOH window.
                for _ in range(int(step_counts[k])):
                    env.step(zero_action)

        quat_xyzw = quat_normalize_xyzw(quat_xyzw)
        e_pos = x_des - x
        e_ori = orientation_error_vec_xyzw(quat_des_xyzw, quat_xyzw)

        csv_columns = ["t_s"]
        csv_columns += [f"q{i}" for i in range(1, 8)]
        csv_columns += [f"dq{i}" for i in range(1, 8)]
        csv_columns += ["x_x", "x_y", "x_z", "dx_x", "dx_y", "dx_z"]
        csv_columns += ["quat_x", "quat_y", "quat_z", "quat_w", "wx", "wy", "wz"]
        csv_columns += ["x_des_x", "x_des_y", "x_des_z", "dx_des_x", "dx_des_y", "dx_des_z"]
        csv_columns += ["quat_des_x", "quat_des_y", "quat_des_z", "quat_des_w"]
        csv_columns += ["e_x", "e_y", "e_z", "e_ox", "e_oy", "e_oz"]

        csv_data = np.column_stack(
            (t_s, q, dq, x, dx, quat_xyzw, w, x_des, dx_des, quat_des_xyzw, e_pos, e_ori)
        )

        os.makedirs(out_csv_path.parent, exist_ok=True)
        np.savetxt(out_csv_path, csv_data, delimiter=",", header=",".join(csv_columns),
                   comments="", fmt="%.9f")

        args_used = dict(real_args)
        args_used["kp_pos"] = kp_pos
        args_used["kp_ori"] = kp_ori
        args_used["kd_pos"] = 2.0 * math.sqrt(kp_pos)
        args_used["kd_ori"] = 2.0 * math.sqrt(kp_ori)
        args_used["zoh_mean_steps_per_csv_row"] = float(step_counts.mean())
        args_used["zoh_total_phys_steps"] = total_phys_steps

        summary = compute_summary(e_pos, e_ori, t_s)
        sidecar_payload = {
            "schema_version": 1,
            "controller": "uw_franka_replay_python_sim",
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "csv_path": str(out_csv_path),
            "sidecar_path": str(out_sidecar_path),
            "source": {
                "real_csv": str(real_csv_path),
                "real_sidecar": str(real_sidecar_path),
                "real_controller": real_sidecar.get("controller"),
                "real_control_rate_hz": real_sidecar.get("control_rate_hz"),
            },
            "sim": {
                "engine": "isaac_lab",
                "robot_asset": str(unwrapped.cfg.robot.spawn.usd_path),
                "integrator_hz": int(round(1.0 / (unwrapped.physics_dt * unwrapped.cfg.decimation))),
                "gravity_handling": "engine_auto",
                "friction_model": "implicit_actuator_friction=0.0",
                "controller": "franka_sysid.control",
                "control_mode": str(unwrapped.cfg.control_mode),
                "use_nullspace": bool(unwrapped.cfg.use_nullspace),
                "gripper_open_width": float(unwrapped.cfg.gripper_open_width),
                "sysid_params_path": None if sysid_params is None else sysid_params["source_path"],
                "sysid_motor_delay_steps": None if sysid_params is None else int(sysid_params["motor_delay_steps"]),
                "zoh_replay": True,
            },
            "args": args_used,
            "q_init": q_init,
            "x_anchor": x_anchor.tolist(),
            "q_anchor_xyzw": q_anchor_xyzw.tolist(),
            "abort": {"code": 0, "name": "none", "joint": -1, "value": 0.0, "time_s": 0.0},
            "summary": summary,
            "exception": None,
            "ended_normally": True,
        }
        os.makedirs(out_sidecar_path.parent, exist_ok=True)
        with open(out_sidecar_path, "w", encoding="utf-8") as f:
            json.dump(sidecar_payload, f, indent=2)

        print(f"[replay_python_csv_sim] wrote sim csv: {out_csv_path}")
        print(f"[replay_python_csv_sim] wrote sim sidecar: {out_sidecar_path}")
        print(f"[replay_python_csv_sim] err_pos_rms_xyz={summary['err_pos_rms_xyz']} "
              f"err_ori_rms_xyz={summary['err_ori_rms_xyz']}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
