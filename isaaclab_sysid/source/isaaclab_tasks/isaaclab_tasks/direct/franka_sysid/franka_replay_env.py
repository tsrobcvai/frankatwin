# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

from . import control as uw_control
from .franka_replay_env_cfg import FrankaTwinReplayEnvCfg


class FrankaTwinReplayEnv(DirectRLEnv):
    """Franka-only replay environment for sim2real trajectory matching."""

    cfg: FrankaTwinReplayEnvCfg

    def __init__(self, cfg: FrankaTwinReplayEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.pos_threshold = torch.tensor(self.cfg.ctrl.pos_action_threshold, device=self.device).repeat((self.num_envs, 1))
        self.rot_threshold = torch.tensor(self.cfg.ctrl.rot_action_threshold, device=self.device).repeat((self.num_envs, 1))

        task_prop_gains = torch.tensor(self.cfg.ctrl.operation_space_cfg.task_prop_gains, device=self.device).repeat(
            (self.num_envs, 1)
        )
        task_deriv_scale = self.cfg.ctrl.operation_space_cfg.task_deriv_scale
        self.task_prop_gains = task_prop_gains.float()
        self.task_deriv_gains = uw_control.get_deriv_gains(self.task_prop_gains, task_deriv_scale)

        self.ctrl_target_joint_pos = torch.zeros((self.num_envs, self._robot.num_joints), device=self.device)
        # Hold the gripper open at its initial width; we never command a close.
        self.ctrl_target_joint_pos[:, 7:9] = float(self.cfg.gripper_open_width)
        self.ctrl_target_fingertip_midpoint_pos = torch.zeros((self.num_envs, 3), device=self.device)
        self.ctrl_target_fingertip_midpoint_quat = (
            torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).unsqueeze(0).repeat(self.num_envs, 1)
        )
        self.dead_zone_thresholds = None

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
        self.applied_wrench = torch.zeros((self.num_envs, 6), device=self.device)

    def _setup_scene(self):
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg(), translation=(0.0, 0.0, -1.05))
        self._robot = Articulation(self.cfg.robot)

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions()
        self.scene.articulations["robot"] = self._robot

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

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

        self.joint_torque, self.applied_wrench = uw_control.compute_dof_torque(
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

        # Arm: send pure torque (implicit actuator has stiffness/damping=0).
        # Gripper: keep at the open width via the implicit PD; never command close.
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

    def set_target_pose(self, pos_w_b: torch.Tensor, quat_w_b_wxyz: torch.Tensor):
        pos_w_b = torch.as_tensor(pos_w_b, dtype=torch.float32, device=self.device)
        quat_w_b_wxyz = torch.as_tensor(quat_w_b_wxyz, dtype=torch.float32, device=self.device)

        if pos_w_b.ndim == 1:
            pos_w_b = pos_w_b.unsqueeze(0)
        if quat_w_b_wxyz.ndim == 1:
            quat_w_b_wxyz = quat_w_b_wxyz.unsqueeze(0)

        if pos_w_b.shape != (self.num_envs, 3):
            raise ValueError(f"Expected target position shape {(self.num_envs, 3)}, got {tuple(pos_w_b.shape)}.")
        if quat_w_b_wxyz.shape != (self.num_envs, 4):
            raise ValueError(f"Expected target quaternion shape {(self.num_envs, 4)}, got {tuple(quat_w_b_wxyz.shape)}.")

        quat_norm = torch.linalg.norm(quat_w_b_wxyz, dim=-1, keepdim=True).clamp_min(1e-12)
        self.ctrl_target_fingertip_midpoint_pos[:] = pos_w_b
        self.ctrl_target_fingertip_midpoint_quat[:] = quat_w_b_wxyz / quat_norm

    def get_state_snapshot(self) -> dict:
        if self.last_update_timestamp < self._robot._data._sim_timestamp:
            self._compute_intermediate_values(dt=self.physics_dt)

        return {
            "sim_time_s": float(self._robot._data._sim_timestamp),
            "q": self.joint_pos[0, :7].detach().cpu().numpy(),
            "dq": self.joint_vel[0, :7].detach().cpu().numpy(),
            "x": self.fingertip_midpoint_pos[0].detach().cpu().numpy(),
            "dx": self.fingertip_midpoint_linvel[0].detach().cpu().numpy(),
            "quat_wxyz": self.fingertip_midpoint_quat[0].detach().cpu().numpy(),
            "w": self.fingertip_midpoint_angvel[0].detach().cpu().numpy(),
            "target_pos": self.ctrl_target_fingertip_midpoint_pos[0].detach().cpu().numpy(),
            "target_quat_wxyz": self.ctrl_target_fingertip_midpoint_quat[0].detach().cpu().numpy(),
        }
