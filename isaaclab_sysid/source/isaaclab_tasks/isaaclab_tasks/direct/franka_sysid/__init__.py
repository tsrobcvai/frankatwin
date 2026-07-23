# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Franka system-identification and replay environments for sim-to-real transfer.

Companion tasks for the `panda_control` real-robot stack. Zero-reward /
zero-observation Franka-only envs driven by a 6-DOF task-impedance controller
that mirrors the real `osc_shm` controller, used to replay real trajectories
and fit arm dynamics (armature / friction / motor delay) with CMA-ES.
"""

import gymnasium as gym

from .franka_replay_env import UWFrankaReplayEnv
from .franka_replay_env_cfg import UWFrankaReplayEnvCfg
from .franka_sysid_env import UWFrankaSysidEnv
from .franka_sysid_env_cfg import UWFrankaSysidEnvCfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-UW-Franka-Replay-v0",
    entry_point="isaaclab_tasks.direct.franka_sysid.franka_replay_env:UWFrankaReplayEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": UWFrankaReplayEnvCfg,
    },
)

gym.register(
    id="Isaac-UW-Franka-Sysid-v0",
    entry_point="isaaclab_tasks.direct.franka_sysid.franka_sysid_env:UWFrankaSysidEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": UWFrankaSysidEnvCfg,
    },
)
