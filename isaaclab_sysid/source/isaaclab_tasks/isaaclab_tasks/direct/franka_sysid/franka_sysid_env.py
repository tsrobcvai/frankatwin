# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

from . import control as uw_control
from .franka_sysid_env_cfg import FrankaTwinSysidEnvCfg


class FrankaTwinSysidEnv(DirectRLEnv):
    """Franka-only environment for CMA-ES system identification."""

    cfg: FrankaTwinSysidEnvCfg

    def __init__(self, cfg: FrankaTwinSysidEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        task_prop_gains = torch.tensor(self.cfg.ctrl.operation_space_cfg.task_prop_gains, device=self.device).repeat(
            (self.num_envs, 1)
        )
        task_deriv_scale = self.cfg.ctrl.operation_space_cfg.task_deriv_scale
        self.task_prop_gains = task_prop_gains.float()
        self.task_deriv_gains = uw_control.get_deriv_gains(self.task_prop_gains, task_deriv_scale)

        self.ctrl_target_joint_pos = torch.zeros((self.num_envs, self._robot.num_joints), device=self.device)
        self.ctrl_target_joint_pos[:, 7:9] = float(self.cfg.gripper_open_width)
        self.ctrl_target_fingertip_midpoint_pos = torch.zeros((self.num_envs, 3), device=self.device)
        self.ctrl_target_fingertip_midpoint_quat = (
            torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).unsqueeze(0).repeat(self.num_envs, 1)
        )

        self.left_finger_body_idx = self._robot.body_names.index("panda_leftfinger")
        self.right_finger_body_idx = self._robot.body_names.index("panda_rightfinger")
        self.fingertip_body_idx = self._robot.body_names.index("panda_fingertip_centered")

        self.last_update_timestamp = 0.0
        self.fingertip_midpoint_pos = torch.zeros((self.num_envs, 3), device=self.device)
        self.fingertip_midpoint_quat = (
            torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).unsqueeze(0).repeat(self.num_envs, 1)
        )
        self.fingertip_midpoint_linvel = torch.zeros((self.num_envs, 3), device=self.device)
        self.fingertip_midpoint_angvel = torch.zeros((self.num_envs, 3), device=self.device)
        self.fingertip_midpoint_jacobian = torch.zeros((self.num_envs, 6, 7), device=self.device)
        self.arm_mass_matrix = torch.zeros((self.num_envs, 7, 7), device=self.device)
        self.joint_pos = torch.zeros((self.num_envs, self._robot.num_joints), device=self.device)
        self.joint_vel = torch.zeros((self.num_envs, self._robot.num_joints), device=self.device)
        self.joint_torque = torch.zeros((self.num_envs, self._robot.num_joints), device=self.device)

        self._target_pos_traj: torch.Tensor | None = None
        self._target_dx_traj: torch.Tensor | None = None
        self._target_quat_traj: torch.Tensor | None = None
        self._target_len: int = 0
        self._zero_action = torch.zeros((self.num_envs, int(getattr(self.cfg, "action_space", 0) or 0)), device=self.device)

        if self.cfg.traj_npz_path:
            self._load_targets_from_npz(self.cfg.traj_npz_path)

    def _setup_scene(self):
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg(), translation=(0.0, 0.0, -1.05))
        self._robot = Articulation(self.cfg.robot)

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions()
        self.scene.articulations["robot"] = self._robot

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _load_targets_from_npz(self, npz_path: str):
        data = np.load(npz_path)
        x_des = data["x_des"]
        dx_des = data["dx_des"] if "dx_des" in data else np.zeros_like(x_des)
        if "quat_des_wxyz" in data:
            quat_des = data["quat_des_wxyz"]
        elif "quat_des_xyzw" in data:
            q_xyzw = data["quat_des_xyzw"]
            quat_des = np.stack((q_xyzw[:, 3], q_xyzw[:, 0], q_xyzw[:, 1], q_xyzw[:, 2]), axis=1)
        else:
            raise KeyError("traj npz requires 'quat_des_wxyz' or 'quat_des_xyzw'.")
        self.set_targets(x_des, dx_des, quat_des)

    def set_targets(self, x_des_T, dx_des_T, quat_des_T):
        x_des = torch.as_tensor(x_des_T, dtype=torch.float32, device=self.device)
        dx_des = torch.as_tensor(dx_des_T, dtype=torch.float32, device=self.device)
        quat_des = torch.as_tensor(quat_des_T, dtype=torch.float32, device=self.device)

        if x_des.ndim != 2 or x_des.shape[1] != 3:
            raise ValueError(f"x_des_T must be (T,3), got {tuple(x_des.shape)}")
        if dx_des.ndim != 2 or dx_des.shape[1] != 3:
            raise ValueError(f"dx_des_T must be (T,3), got {tuple(dx_des.shape)}")
        if quat_des.ndim != 2 or quat_des.shape[1] != 4:
            raise ValueError(f"quat_des_T must be (T,4) wxyz, got {tuple(quat_des.shape)}")
        if x_des.shape[0] != quat_des.shape[0]:
            raise ValueError("x_des and quat_des must share T")

        quat_norm = torch.linalg.norm(quat_des, dim=-1, keepdim=True).clamp_min(1e-12)
        self._target_pos_traj = x_des
        self._target_dx_traj = dx_des
        self._target_quat_traj = quat_des / quat_norm
        self._target_len = int(x_des.shape[0])

    def _set_target_from_index(self, t_idx: int):
        if self._target_pos_traj is None or self._target_quat_traj is None:
            raise RuntimeError("Targets are not set. Call set_targets() first.")
        if t_idx < 0 or t_idx >= self._target_len:
            raise IndexError(f"t_idx={t_idx} is out of range [0, {self._target_len}).")
        pos = self._target_pos_traj[t_idx].unsqueeze(0).repeat(self.num_envs, 1)
        quat = self._target_quat_traj[t_idx].unsqueeze(0).repeat(self.num_envs, 1)
        self.ctrl_target_fingertip_midpoint_pos[:] = pos
        self.ctrl_target_fingertip_midpoint_quat[:] = quat

    def step_replay(self, t_idx: int) -> dict[str, torch.Tensor]:
        """Advance one 1kHz step using target row `t_idx`."""
        self._set_target_from_index(t_idx)
        self.step(self._zero_action)
        # Official IsaacLab's DirectRLEnv has no `_post_physics_step` hook, so the
        # buffers may still hold pre-step state here; refresh before sampling.
        if self.last_update_timestamp < self._robot._data._sim_timestamp:
            self._compute_intermediate_values(dt=self.physics_dt)
        return {
            "joint_pos": self.joint_pos[:, :7].clone(),
            "joint_vel": self.joint_vel[:, :7].clone(),
            "fingertip_pos": self.fingertip_midpoint_pos.clone(),
            "fingertip_quat_wxyz": self.fingertip_midpoint_quat.clone(),
        }

    def _pre_physics_step(self, action):
        del action
        if self.last_update_timestamp < self._robot._data._sim_timestamp:
            self._compute_intermediate_values(dt=self.physics_dt)

    def _post_physics_step(self):
        if self.last_update_timestamp < self._robot._data._sim_timestamp:
            self._compute_intermediate_values(dt=self.physics_dt)

    def _apply_action(self):
        if self.last_update_timestamp < self._robot._data._sim_timestamp:
            self._compute_intermediate_values(dt=self.physics_dt)

        self.joint_torque, _ = uw_control.compute_dof_torque(
            dof_pos=self.joint_pos,
            dof_vel=self.joint_vel,
            fingertip_pos=self.fingertip_midpoint_pos,
            fingertip_quat=self.fingertip_midpoint_quat,
            fingertip_linvel=self.fingertip_midpoint_linvel,
            fingertip_angvel=self.fingertip_midpoint_angvel,
            jacobian=self.fingertip_midpoint_jacobian,
            arm_mass_matrix=self.arm_mass_matrix,
            target_pos=self.ctrl_target_fingertip_midpoint_pos,
            target_quat=self.ctrl_target_fingertip_midpoint_quat,
            task_prop_gains=self.task_prop_gains,
            task_deriv_gains=self.task_deriv_gains,
            control_mode=str(self.cfg.control_mode),
            use_nullspace=bool(self.cfg.use_nullspace),
            default_dof_pos=self.cfg.default_dof_pos,
            kp_null=float(self.cfg.kp_null),
            kd_null=float(self.cfg.kd_null),
            torque_clamp=float(self.cfg.torque_clamp_nm),
            device=self.device,
        )

        self.ctrl_target_joint_pos[:, :7] = self.joint_pos[:, :7]
        self.ctrl_target_joint_pos[:, 7:9] = float(self.cfg.gripper_open_width)
        self.joint_torque[:, 7:9] = 0.0
        self._robot.set_joint_position_target(self.ctrl_target_joint_pos)
        self._robot.set_joint_effort_target(self.joint_torque)

    def _compute_intermediate_values(self, dt: float):
        del dt
        self.fingertip_midpoint_pos = self._robot.data.body_pos_w[:, self.fingertip_body_idx] - self.scene.env_origins
        self.fingertip_midpoint_quat = self._robot.data.body_quat_w[:, self.fingertip_body_idx]
        self.fingertip_midpoint_linvel = self._robot.data.body_lin_vel_w[:, self.fingertip_body_idx]
        self.fingertip_midpoint_angvel = self._robot.data.body_ang_vel_w[:, self.fingertip_body_idx]

        jacobians = self._robot.root_physx_view.get_jacobians()
        left_finger_jacobian = jacobians[:, self.left_finger_body_idx - 1, 0:6, 0:7]
        right_finger_jacobian = jacobians[:, self.right_finger_body_idx - 1, 0:6, 0:7]
        self.fingertip_midpoint_jacobian = (left_finger_jacobian + right_finger_jacobian) * 0.5

        self.arm_mass_matrix = self._robot.root_physx_view.get_generalized_mass_matrices()[:, 0:7, 0:7]
        self.joint_pos = self._robot.data.joint_pos.clone()
        self.joint_vel = self._robot.data.joint_vel.clone()
        self.last_update_timestamp = self._robot._data._sim_timestamp

    def _get_observations(self):
        return {"policy": torch.zeros((self.num_envs, 0), device=self.device)}

    def _get_dones(self):
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return time_out, time_out

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)

        joint_pos = self._robot.data.default_joint_pos[env_ids].clone()
        if self.cfg.q_init is not None:
            if len(self.cfg.q_init) != 7:
                raise ValueError(f"Expected 7 values in q_init, got {len(self.cfg.q_init)}.")
            joint_pos[:, :7] = torch.tensor(self.cfg.q_init, device=self.device)[None, :]
        else:
            joint_pos[:, :7] = torch.tensor(self.cfg.ctrl.reset_joints, device=self.device)[None, :]
        joint_pos[:, 7:9] = float(self.cfg.gripper_open_width)

        joint_vel = torch.zeros_like(joint_pos)
        joint_effort = torch.zeros_like(joint_pos)
        self._robot.set_joint_position_target(joint_pos.clone(), env_ids=env_ids)
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
        self._robot.reset()
        self._robot.set_joint_effort_target(joint_effort, env_ids=env_ids)

        self.step_sim_no_action()
        self.ctrl_target_fingertip_midpoint_pos[:] = self.fingertip_midpoint_pos
        self.ctrl_target_fingertip_midpoint_quat[:] = self.fingertip_midpoint_quat
        self.actions[env_ids] = torch.zeros_like(self.actions[env_ids])

    def _get_rewards(self):
        return torch.zeros((self.num_envs,), device=self.device)

    def step_sim_no_action(self):
        self.scene.write_data_to_sim()
        self.sim.step(render=False)
        self.scene.update(dt=self.physics_dt)
        self._compute_intermediate_values(dt=self.physics_dt)
