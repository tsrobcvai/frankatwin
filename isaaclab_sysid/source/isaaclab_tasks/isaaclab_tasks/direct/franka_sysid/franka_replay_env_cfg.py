# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for ``Isaac-FrankaTwin-Replay-v0``.

1 ms physics with ``decimation = 1``. ``ctrl`` carries the impedance gains,
``q_init`` the start configuration and ``traj_log_path`` the sim CSV to write;
``control_mode`` and ``use_nullspace`` select the controller variant. The arm
joints use ``ImplicitActuatorCfg`` — a replay has no actuator parameters to
identify, which is what separates this config from
:mod:`franka_sysid_env_cfg`.
"""


import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import PhysxCfg, SimulationCfg
from isaaclab.sim.spawners.materials.physics_materials_cfg import RigidBodyMaterialCfg
from isaaclab.utils import configclass

from .ctrl_cfg import CtrlCfg, OSCCtrlCfg


class ReplayOSCCtrlCfg(OSCCtrlCfg):
    # Match real sidecar defaults: kp_pos=200, kp_ori=20.
    task_prop_gains = [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]
    task_deriv_scale = 1.0


@configclass
class ReplayCtrlCfg(CtrlCfg):
    operation_space_cfg = ReplayOSCCtrlCfg()


@configclass
class FrankaTwinReplayEnvCfg(DirectRLEnvCfg):
    decimation = 1
    episode_length_s = 8.0
    action_space = 0
    observation_space = 0
    state_space = 0

    ctrl: ReplayCtrlCfg = ReplayCtrlCfg()

    # Optional runner-injected initial joint configuration (7 arm joints).
    q_init: list[float] | None = None
    traj_log_path: str | None = None

    # Cartesian controller mode.
    #   - "task_impedance": tau = J^T @ f_task (matches the real osc_shm)
    #   - "osc":            tau = J^T @ Λ @ f_task (classic operational space)
    control_mode: str = "task_impedance"

    # OSC nullspace toggle. Default OFF to match the real-side osc_shm
    # controller (`frankatwin/src/osc_shm.cpp`), which has no
    # nullspace term.  When enabled, joints are pulled toward `default_dof_pos`.
    use_nullspace: bool = False
    default_dof_pos: list[float] = [
        0.09017809387254755,
        -0.9824203501652151,
        0.030509718397568178,
        -2.694229634937343,
        0.057700675144720104,
        1.860298714876101,
        0.8713759453244422,
    ]
    kp_null: float = 1.0
    kd_null: float = 0.0
    torque_clamp_nm: float = 100.0

    # Gripper is held open at this width (rad/m for each finger joint).
    gripper_open_width: float = 0.04

    sim: SimulationCfg = SimulationCfg(
        device="cuda:0",
        dt=1 / 1000,
        gravity=(0.0, 0.0, -9.81),
        physx=PhysxCfg(
            solver_type=1,
            max_position_iteration_count=192,
            max_velocity_iteration_count=1,
            bounce_threshold_velocity=0.2,
            friction_offset_threshold=0.01,
            friction_correlation_distance=0.00625,
            gpu_max_rigid_contact_count=2**23,
            gpu_max_rigid_patch_count=2**23,
            gpu_collision_stack_size=2**28,
            gpu_max_num_partitions=1,
        ),
        physics_material=RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        render=sim_utils.RenderCfg(
            rendering_mode="balanced",
            carb_settings={"rtx.reflections.enabled": True},
        ),
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1,
        env_spacing=2.0,
        replicate_physics=False,
        clone_in_fabric=False,
    )

    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path="./source/isaaclab_assets/data/Robots/Franka/franka_mimic.usd",
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
                linear_damping=0.01,
                angular_damping=0.01,
                max_linear_velocity=1000.0,
                max_angular_velocity=3666.0,
                enable_gyroscopic_forces=True,
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=1,
                max_contact_impulse=1e32,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=1,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "panda_joint1": 0.00871,
                "panda_joint2": -0.10368,
                "panda_joint3": -0.00794,
                "panda_joint4": -1.49139,
                "panda_joint5": -0.00083,
                "panda_joint6": 1.38774,
                "panda_joint7": 0.82,
                "panda_finger_joint2": 0.04,
            },
        ),
        actuators={
            "panda_arm1": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint[1-4]"],
                stiffness=0.0,
                damping=0.0,
                friction=0.0,
                armature=0.0,
                effort_limit_sim=100,
                velocity_limit_sim=124.6,
            ),
            "panda_arm2": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint[5-7]"],
                stiffness=0.0,
                damping=0.0,
                friction=0.0,
                armature=0.0,
                effort_limit_sim=50,
                velocity_limit_sim=149.5,
            ),
            "panda_hand": ImplicitActuatorCfg(
                joint_names_expr=["panda_finger_joint[1-2]"],
                effort_limit_sim=100.0,
                velocity_limit_sim=0.1,
                stiffness=7500.0,
                damping=173.0,
                friction=5.0,
                armature=0.0,
            ),
        },
    )
