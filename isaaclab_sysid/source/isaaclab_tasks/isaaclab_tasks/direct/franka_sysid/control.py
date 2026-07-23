# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Cartesian controller for `Isaac-UW-Franka-Replay-v0`.

Two task-space modes are supported, selectable via ``control_mode``:

* ``"task_impedance"`` (default) - matches the real step5b controller in
  `panda_control/src/step5b_cart_pose.cpp` (see
  `panda_control/SIM2REAL_COMPARISON.md` §4) **exactly**::

      f_task[0:3] = Kp_pos * (x_des - x) - Kd_pos * v
      f_task[3:6] = Kp_ori * e_o          - Kd_ori * w
      tau        = J^T @ f_task

  No apparent-mass projection.  The Cartesian gains are interpreted as
  task-space stiffness / damping directly.

* ``"osc"`` - classic operational-space control with the
  ``Λ = (J M^-1 J^T)^-1`` apparent-mass projection::

      tau = J^T @ Λ @ f_task

  Use this only when you intentionally want the Isaac Lab default
  behaviour; it will *not* match the real controller bit-for-bit.

A nullspace projection (pulling joints toward ``default_dof_pos``) is
available in both modes via ``use_nullspace``.  It is OFF by default so
the controller matches the real Franka controller, which has no
nullspace term.
"""

from __future__ import annotations

import math
from typing import Sequence

import torch

import isaacsim.core.utils.torch as torch_utils
from isaaclab.utils.math import axis_angle_from_quat


def compute_pose_error(
    fingertip_pos: torch.Tensor,
    fingertip_quat: torch.Tensor,
    target_pos: torch.Tensor,
    target_quat: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return (position_error, axis_angle_error) using shortest-path quaternion handling.

    All quaternions are in `wxyz` order (IsaacLab convention).
    """
    pos_error = target_pos - fingertip_pos

    quat_dot = (target_quat * fingertip_quat).sum(dim=1, keepdim=True)
    target_quat_short = torch.where(
        quat_dot.expand(-1, 4) >= 0,
        target_quat,
        -target_quat,
    )

    fingertip_quat_inv = torch_utils.quat_conjugate(fingertip_quat)
    quat_error = torch_utils.quat_mul(target_quat_short, fingertip_quat_inv)
    axis_angle_error = axis_angle_from_quat(quat_error)
    return pos_error, axis_angle_error


def compute_dof_torque(
    *,
    dof_pos: torch.Tensor,
    dof_vel: torch.Tensor,
    fingertip_pos: torch.Tensor,
    fingertip_quat: torch.Tensor,
    fingertip_linvel: torch.Tensor,
    fingertip_angvel: torch.Tensor,
    jacobian: torch.Tensor,
    arm_mass_matrix: torch.Tensor,
    target_pos: torch.Tensor,
    target_quat: torch.Tensor,
    task_prop_gains: torch.Tensor,
    task_deriv_gains: torch.Tensor,
    control_mode: str = "task_impedance",
    use_nullspace: bool = False,
    default_dof_pos: Sequence[float] | None = None,
    kp_null: float = 0.0,
    kd_null: float = 0.0,
    torque_clamp: float = 100.0,
    device: torch.device | str | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Compute joint torques for an OSC pose-tracking controller.

    Args:
        dof_pos, dof_vel: full robot DOF state, shape (N, num_dof).
        fingertip_pos, fingertip_quat, fingertip_linvel, fingertip_angvel:
            current EE state in base frame, shape (N, 3 or 4).
        jacobian: 6x7 geometric Jacobian, shape (N, 6, 7).
        arm_mass_matrix: 7x7 generalized inertia of the arm, shape (N, 7, 7).
        target_pos, target_quat: desired EE pose, shape (N, 3) / (N, 4) wxyz.
        task_prop_gains, task_deriv_gains: (N, 6) PD gains in task space.
        control_mode: ``"task_impedance"`` (default, matches real step5b) or
            ``"osc"`` (apparent-mass projection).
        use_nullspace: if True, add nullspace torque pulling joints toward
            `default_dof_pos`. Default False (matches real controller).
        default_dof_pos: 7-element joint reference for nullspace projection.
        kp_null, kd_null: nullspace PD gains (only used when use_nullspace).
        torque_clamp: per-joint torque clamp magnitude (Nm).
        device: torch device for the output tensor.

    Returns:
        dof_torque: (N, num_dof) torque tensor. Only the first 7 entries
            (arm joints) are populated; finger entries are zero.
        task_wrench: (N, 6) OSC-projected task wrench applied at the EE.
    """
    num_envs = dof_pos.shape[0]
    out_device = device if device is not None else dof_pos.device

    pos_error, axis_angle_error = compute_pose_error(
        fingertip_pos=fingertip_pos,
        fingertip_quat=fingertip_quat,
        target_pos=target_pos,
        target_quat=target_quat,
    )

    # Task-space PD wrench (motion). Matches `f_task` in the real controller.
    task_wrench_motion = torch.zeros((num_envs, 6), device=out_device, dtype=dof_pos.dtype)
    task_wrench_motion[:, 0:3] = (
        task_prop_gains[:, 0:3] * pos_error - task_deriv_gains[:, 0:3] * fingertip_linvel
    )
    task_wrench_motion[:, 3:6] = (
        task_prop_gains[:, 3:6] * axis_angle_error - task_deriv_gains[:, 3:6] * fingertip_angvel
    )

    jacobian_T = torch.transpose(jacobian, 1, 2)

    if control_mode == "task_impedance":
        # Pure task-space PD: `task_wrench = f_task` (no apparent mass).
        # This is what `panda_control/src/step5b_cart_pose.cpp` implements.
        task_wrench = task_wrench_motion
    elif control_mode == "osc":
        # Apparent-mass projection: ETH eq. 3.86, classic operational-space.
        arm_mass_matrix_inv = torch.inverse(arm_mass_matrix)
        arm_mass_matrix_task = torch.inverse(jacobian @ arm_mass_matrix_inv @ jacobian_T)
        task_wrench = (arm_mass_matrix_task @ task_wrench_motion.unsqueeze(-1)).squeeze(-1)
    else:
        raise ValueError(f"Unknown control_mode {control_mode!r}; expected 'task_impedance' or 'osc'.")

    dof_torque = torch.zeros((num_envs, dof_pos.shape[1]), device=out_device, dtype=dof_pos.dtype)
    dof_torque[:, 0:7] = (jacobian_T @ task_wrench.unsqueeze(-1)).squeeze(-1)

    if use_nullspace:
        if default_dof_pos is None:
            raise ValueError("use_nullspace=True requires `default_dof_pos`.")
        if len(default_dof_pos) != 7:
            raise ValueError(f"default_dof_pos must have 7 elements, got {len(default_dof_pos)}.")

        # Nullspace projection needs the apparent-mass terms regardless of mode.
        arm_mass_matrix_inv_ns = torch.inverse(arm_mass_matrix)
        arm_mass_matrix_task_ns = torch.inverse(jacobian @ arm_mass_matrix_inv_ns @ jacobian_T)
        j_eef_inv = arm_mass_matrix_task_ns @ jacobian @ arm_mass_matrix_inv_ns
        default_q = torch.as_tensor(default_dof_pos, device=out_device, dtype=dof_pos.dtype).repeat((num_envs, 1))

        distance = default_q - dof_pos[:, :7]
        distance = (distance + math.pi) % (2.0 * math.pi) - math.pi

        u_null = kd_null * -dof_vel[:, :7] + kp_null * distance
        u_null = arm_mass_matrix @ u_null.unsqueeze(-1)
        torque_null = (
            torch.eye(7, device=out_device, dtype=dof_pos.dtype).unsqueeze(0) - jacobian_T @ j_eef_inv
        ) @ u_null
        dof_torque[:, 0:7] += torque_null.squeeze(-1)

    dof_torque = torch.clamp(dof_torque, min=-torque_clamp, max=torque_clamp)
    return dof_torque, task_wrench


def get_deriv_gains(prop_gains, rot_deriv_scale=1.0):
    """Set robot gains using critical damping."""
    deriv_gains = 2 * torch.sqrt(prop_gains)
    deriv_gains[:, 3:6] /= rot_deriv_scale
    return deriv_gains
