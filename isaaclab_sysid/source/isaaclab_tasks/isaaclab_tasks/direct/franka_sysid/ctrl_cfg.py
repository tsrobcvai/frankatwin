# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Controller configuration shared by the sysid / replay Franka envs."""

from isaaclab.utils import configclass


class OSCCtrlCfg:
    task_prop_gains = [450, 450, 450, 850, 850, 850]
    task_deriv_scale = 1.0


@configclass
class CtrlCfg:
    pos_action_threshold = [0.01, 0.01, 0.01]
    rot_action_threshold = [0.04, 0.04, 0.04]

    reset_joints = [-1.3003, -0.4015, 1.1791, -2.1493, 0.4001, 1.9425, 0.4754]

    operation_space_cfg = OSCCtrlCfg()
