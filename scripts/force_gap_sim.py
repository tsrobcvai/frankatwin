#!/usr/bin/env python3
"""Collect a physical-grasp wrist-wrench experiment in Isaac Lab 3 / PhysX.

Run with Isaac Lab's Python environment. No installation into Isaac Lab is needed.
"""
from __future__ import annotations

import argparse
import hashlib
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from frankatwin.force_gap.protocol import Protocol

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--protocol", type=Path)
parser.add_argument(
    "--real-run",
    type=Path,
    help="Replay the exact logged real setpoints and K/EE frames.",
)
parser.add_argument(
    "--robot-usd",
    type=Path,
    default=ROOT
    / "isaaclab_sysid/source/isaaclab_assets/data/Robots/Franka/franka_mimic.usd",
)
parser.add_argument(
    "--empty",
    action="store_true",
    help="Record the empty gripper at the object grasp width.",
)
parser.add_argument("--dt", type=float, default=0.001)
parser.add_argument(
    "--duration",
    type=float,
    help="Debug truncation; a truncated run is marked incomplete.",
)
parser.add_argument("--video", action="store_true")
parser.add_argument("--video-fps", type=int, default=20)
parser.add_argument("--warmup", type=float, default=2.0)
from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = args.video or args.enable_cameras
args.out = args.out.resolve()
args.robot_usd = args.robot_usd.resolve()
args.out.mkdir(parents=True, exist_ok=False)
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import imageio.v2 as imageio
import numpy as np
import torch
import warp as wp
from pxr import Usd, UsdPhysics
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
from isaaclab.sensors import JointWrenchSensor, JointWrenchSensorCfg
from isaaclab_assets import FRANKA_PANDA_CFG
from isaaclab_physx.physics import PhysxCfg
from frankatwin.force_gap.math import (
    rotation,
    multiply,
    pose_matrix,
    transform_wrench,
    support_wrench,
    derivative,
    box_inertia,
    quaternion_angle,
    validate_transform,
    matrix_quaternion,
)
from frankatwin.force_gap.data import (
    save_bundle,
    derive_frames,
    write_json,
    load_bundle,
)


def tensor(value):
    if isinstance(value, torch.Tensor):
        return value
    if hasattr(value, "torch"):
        return value.torch
    return wp.to_torch(value)


def array(value):
    return tensor(value).detach().cpu().numpy().copy()


def matrix_quat(r):
    return matrix_quaternion(r)


def poses(value):
    """Isaac Lab 3 exposes xyzw; bundles and libfranka transport use wxyz."""
    a = array(value)
    a[..., 3:] = a[..., [6, 3, 4, 5]]
    return a


def main():
    protocol = Protocol.load(args.protocol)
    replay, real_meta = None, None
    if args.real_run:
        replay, real_meta = load_bundle(args.real_run)
        if real_meta["source"] != "real" or not real_meta.get("completed"):
            raise ValueError("--real-run requires a completed real collection")
        if bool(real_meta["empty"]) != args.empty:
            raise ValueError("match --empty to the real collection load condition")
        protocol = Protocol(**real_meta["protocol"]).validate()
    if (
        args.dt <= 0
        or args.warmup < 0
        or args.video_fps <= 0
        or (args.duration is not None and args.duration <= 0)
    ):
        raise ValueError("invalid simulation timing")
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=args.dt,
            device=args.device,
            gravity=(0.0, 0.0, -9.81),
            physics=PhysxCfg(
                solver_type=1,
                max_position_iteration_count=64,
                max_velocity_iteration_count=8,
            ),
            render=sim_utils.RenderCfg(antialiasing_mode="TAA"),
        )
    )
    sim.set_camera_view([1.15, 1.05, 0.8], [0.30, 0, 0.32])
    ground = sim_utils.GroundPlaneCfg(color=(0.18, 0.20, 0.23))
    ground.func("/World/Ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.95, 0.97, 1.0))
    light.func("/World/Light", light)
    cfg = FRANKA_PANDA_CFG.copy()
    cfg.prim_path = "/World/Robot"
    cfg.spawn.usd_path = str(args.robot_usd)
    cfg.spawn.rigid_props.disable_gravity = False
    cfg.spawn.rigid_props.linear_damping = 0.0
    cfg.spawn.rigid_props.angular_damping = 0.0
    cfg.spawn.rigid_props.enable_gyroscopic_forces = True
    cfg.spawn.articulation_props.enabled_self_collisions = False
    cfg.spawn.articulation_props.solver_position_iteration_count = 64
    cfg.spawn.articulation_props.solver_velocity_iteration_count = 8
    cfg.spawn.collision_props = sim_utils.CollisionPropertiesCfg(
        contact_offset=0.0005, rest_offset=0.0
    )
    cfg.spawn.physics_material = sim_utils.RigidBodyMaterialCfg(
        static_friction=1.0, dynamic_friction=0.8, restitution=0.0
    )
    initial_q = protocol.q_init if replay is None else replay["q"][0].tolist()
    cfg.init_state.joint_pos = {f"panda_joint{i+1}": q for i, q in enumerate(initial_q)}
    cfg.init_state.joint_pos["panda_finger_joint.*"] = protocol.dimensions_m[1] / 2
    for name, act in cfg.actuators.items():
        if name != "panda_hand":
            act.stiffness = 0.0
            act.damping = 0.0
            act.armature = 0.0
        else:
            act.stiffness = 7500.0
            act.damping = 100.0
            act.effort_limit_sim = protocol.grasp_force_n
    robot = Articulation(cfg)
    sensor = JointWrenchSensor(
        JointWrenchSensorCfg(prim_path="/World/Robot", update_period=0.0)
    )
    block_cfg = RigidObjectCfg(
        prim_path="/World/Payload",
        spawn=sim_utils.CuboidCfg(
            size=tuple(protocol.dimensions_m),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False, linear_damping=0.0, angular_damping=0.0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=protocol.mass_kg),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.0005, rest_offset=0.0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.8, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.95, 0.31, 0.06), roughness=0.65
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -2.0)),
    )
    block = RigidObject(block_cfg) if not args.empty else None
    writer, annotator = None, None
    if args.video:
        import omni.replicator.core as rep

        camera = rep.create.camera(
            position=(1.05, 0.95, 0.72), look_at=(0.35, 0.0, 0.32), focal_length=32.0
        )
        product = rep.create.render_product(camera, (960, 720))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([product])
        writer = imageio.get_writer(
            str(args.out / "scene.mp4"),
            fps=args.video_fps,
            codec="libx264",
            quality=None,
            bitrate="4M",
            macro_block_size=16,
        )
    sim.reset()
    arm_ids = robot.find_joints(
        [f"panda_joint{i}" for i in range(1, 8)], preserve_order=True
    )[0]
    finger_ids = robot.find_joints("panda_finger_joint.*")[0]
    wrist_id = robot.body_names.index("force_sensor")
    tip_id = robot.body_names.index("panda_fingertip_centered")
    link7_id = robot.body_names.index("panda_link7")
    # Read the USD child anchor, rather than assuming joint and body origins coincide.
    joint = None
    for prim in Usd.PrimRange(sim.stage.GetPrimAtPath("/World/Robot")):
        j = UsdPhysics.Joint(prim)
        if j and any(p.name == "force_sensor" for p in j.GetBody1Rel().GetTargets()):
            joint = j
            break
    if joint is None:
        raise RuntimeError("no incoming fixed wrist joint in asset")
    pj = joint.GetLocalPos1Attr().Get()
    qj = joint.GetLocalRot1Attr().Get()
    sj = pose_matrix(np.array(pj), np.array([qj.GetReal(), *qj.GetImaginary()]))
    # Descendants below the wrist cut are the modeled tool. The block is a separate rigid body.
    descendants = {"force_sensor"}
    joints = [
        UsdPhysics.Joint(p)
        for p in Usd.PrimRange(sim.stage.GetPrimAtPath("/World/Robot"))
        if p.IsA(UsdPhysics.Joint)
    ]
    for _ in range(len(joints)):
        for j in joints:
            a, b = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
            if a and b and a[0].name in descendants:
                descendants.add(b[0].name)
    tool_ids = [i for i, n in enumerate(robot.body_names) if n in descendants]
    print("[SETUP] bodies=", robot.body_names, "joints=", robot.joint_names, flush=True)
    print(
        "[SETUP] wrist=",
        wrist_id,
        "tool_bodies=",
        [robot.body_names[i] for i in tool_ids],
        flush=True,
    )
    q = tensor(robot.data.default_joint_pos).clone()
    robot.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q))
    robot.reset()
    robot.update(args.dt)
    body_pose = poses(robot.data.body_link_pose_w)[0]
    # Panda flange frame: joint7 link + 107 mm along its z axis.
    link7_to_flange = np.eye(4)
    link7_to_flange[2, 3] = 0.107
    flange = (
        pose_matrix(body_pose[link7_id, :3], body_pose[link7_id, 3:]) @ link7_to_flange
    )
    tip = pose_matrix(body_pose[tip_id, :3], body_pose[tip_id, 3:])
    f_t_ee = np.linalg.inv(flange) @ tip
    ee_t_k = np.eye(4)
    if real_meta:
        f_t_ee = validate_transform(real_meta["F_T_EE"])
        ee_t_k = validate_transform(real_meta["EE_T_K"])
    ee = flange @ f_t_ee
    anchor_pos, anchor_quat = ee[:3, 3].copy(), matrix_quat(ee[:3, :3])
    if real_meta:
        anchor_pos = np.array(real_meta["anchor_pos"])
        anchor_quat = np.array(real_meta["anchor_quat"])
        if (
            np.linalg.norm(ee[:3, 3] - anchor_pos) > 0.01
            or quaternion_angle(matrix_quat(ee[:3, :3]), anchor_quat) > 0.05
        ):
            raise RuntimeError(
                "real/sim initial FK mismatch: check robot asset, q_init and F_T_EE"
            )
    if block:
        bp = ee[:3, 3] + ee[:3, :3] @ np.array(protocol.ee_com_m)
        block.write_root_pose_to_sim_index(
            root_pose=torch.tensor(
                [[*bp, *matrix_quat(ee[:3, :3])[[1, 2, 3, 0]]]],
                device=args.device,
                dtype=torch.float32,
            )
        )
        block.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), device=args.device)
        )
        block.reset()
    masses = array(robot.data.body_mass)[0, tool_ids]
    inertias = array(robot.data.body_inertia)[0, tool_ids].reshape(-1, 3, 3)
    # PhysX tensors are about COM, expressed in the actor's local axes.
    metadata = {
        "source": "sim",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "protocol": protocol.to_dict(),
        "protocol_sha256": protocol.digest,
        "anchor_pos": anchor_pos.tolist(),
        "anchor_quat": anchor_quat.tolist(),
        "F_T_EE": f_t_ee.tolist(),
        "EE_T_K": ee_t_k.tolist(),
        "empty": args.empty,
        "object_in_compensation_model": False,
        "sensor": "JointWrenchSensor",
        "sensor_convention": sensor.cfg.convention,
        "sensor_body": "force_sensor",
        "sensor_child_anchor": sj.tolist(),
        "tool_bodies": [robot.body_names[i] for i in tool_ids],
        "tool_masses": masses.tolist(),
        "tool_inertias": inertias.tolist(),
        "robot_usd": str(args.robot_usd),
        "physics_dt": args.dt,
        "source_sha256": {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                Path(__file__),
                *sorted((ROOT / "python/frankatwin/force_gap").glob("*.py")),
            ]
        },
        "robot_usd_sha256": hashlib.sha256(args.robot_usd.read_bytes()).hexdigest(),
        "initial_q": initial_q,
        "video_fps": args.video_fps,
        "controller": "J^T Cartesian impedance + engine gravity/coriolis compensation; object unmodeled",
        "grasp": "physical finger contacts; no attachment constraint",
        "real_run": None if not args.real_run else str(args.real_run.resolve()),
        "completed": False,
    }
    write_json(args.out / "metadata.json", metadata)
    write_json(args.out / "protocol.json", protocol.to_dict())
    duration = float(replay["t_s"][-1]) if replay is not None else protocol.duration
    requested_duration = duration
    if args.duration is not None:
        duration = min(duration, args.duration)
    rows = []
    frame_times = []
    warmup_steps = round(args.warmup / args.dt)
    total_steps = round(duration / args.dt)
    control_steps = max(1, round(1 / (protocol.control_hz * args.dt)))
    video_steps = max(1, round(1 / (args.video_fps * args.dt)))
    if args.video and abs(video_steps * args.dt * args.video_fps - 1) > 1e-6:
        raise ValueError("video_fps must divide the physics frequency exactly")
    target_pos, target_quat = anchor_pos.copy(), anchor_quat.copy()
    tau_previous = np.zeros(7)
    target_finger = np.full(
        len(finger_ids), protocol.dimensions_m[1] / 2 if args.empty else 0.0
    )
    device = robot.device
    tic = time.monotonic()
    try:
        for step in range(-warmup_steps, total_steps):
            t = max(0.0, step * args.dt)
            bp = poses(robot.data.body_link_pose_w)[0]
            flange = pose_matrix(bp[link7_id, :3], bp[link7_id, 3:]) @ link7_to_flange
            ee = flange @ f_t_ee
            eq = matrix_quat(ee[:3, :3])
            if step >= 0 and (replay is not None or step % control_steps == 0):
                if replay is None:
                    target_pos, target_quat, _, _ = protocol.sample(
                        t, anchor_pos, anchor_quat
                    )
                else:
                    idx = max(0, np.searchsorted(replay["t_s"], t, side="right") - 1)
                    target_pos = replay["target_pos"][idx]
                    target_quat = replay["target_quat"][idx]
            jac = array(robot.data.body_link_jacobian_w)[0, link7_id - 1, :, :][
                :, arm_ids
            ]
            offset = ee[:3, 3] - bp[link7_id, :3]
            jac[:3] += np.cross(jac[3:].T, offset).T
            dq = array(robot.data.joint_vel)[0, arm_ids]
            twist = jac @ dq
            conjugate = eq * np.array([1, -1, -1, -1])
            error = multiply(target_quat, conjugate)
            if error[0] < 0:
                error = -error
            task = np.r_[
                protocol.kp_pos * np.clip(target_pos - ee[:3, 3], -0.04, 0.04)
                - 2 * np.sqrt(protocol.kp_pos) * twist[:3],
                protocol.kp_ori * 2 * error[1:]
                - 2 * np.sqrt(protocol.kp_ori) * twist[3:],
            ]
            cv = array(
                robot.root_view.get_coriolis_and_centrifugal_compensation_forces()
            )[0, arm_ids]
            gravity = array(robot.data.gravity_compensation_forces)[0, arm_ids]
            tau = np.clip(
                jac.T @ task + cv,
                -np.array([87, 87, 87, 87, 12, 12, 12]),
                np.array([87, 87, 87, 87, 12, 12, 12]),
            )
            tau = np.clip(
                tau, tau_previous - 1000 * args.dt, tau_previous + 1000 * args.dt
            )
            tau_previous = tau.copy()
            robot.set_joint_effort_target_index(
                target=torch.tensor(
                    (tau + gravity)[None], device=device, dtype=torch.float32
                ),
                joint_ids=arm_ids,
            )
            robot.set_joint_position_target_index(
                target=torch.tensor(
                    target_finger[None], device=device, dtype=torch.float32
                ),
                joint_ids=finger_ids,
            )
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(args.dt)
            if block:
                block.update(args.dt)
            sensor.update(args.dt, force_recompute=True)
            if step < 0:
                continue
            # Read AFTER the physics step; timestamps refer to the measured state.
            stamp = (step + 1) * args.dt
            bp = poses(robot.data.body_link_pose_w)[0]
            flange = pose_matrix(bp[link7_id, :3], bp[link7_id, 3:]) @ link7_to_flange
            ee = flange @ f_t_ee
            ws = pose_matrix(bp[wrist_id, :3], bp[wrist_id, 3:]) @ sj
            wk = ee @ ee_t_k
            raw = np.r_[
                array(sensor.data.force)[0, wrist_id],
                array(sensor.data.torque)[0, wrist_id],
            ]
            raw_o = transform_wrench(raw, ws[:3, :3], ws[:3, 3] - wk[:3, 3])
            phase, settled = protocol.sample(stamp, anchor_pos, anchor_quat)[2:]
            if replay is not None:
                idx = min(len(replay["t_s"]) - 1, np.searchsorted(replay["t_s"], stamp))
                phase = int(replay["phase"][idx])
                settled = bool(replay["settled"][idx])
            com_pose = poses(robot.data.body_com_pose_w)[0, tool_ids]
            com_vel = array(robot.data.body_com_vel_w)[0, tool_ids]
            object_pose = (
                poses(block.data.root_pose_w)[0]
                if block
                else np.r_[ee[:3, 3], matrix_quat(ee[:3, :3])]
            )
            object_vel = array(block.data.root_vel_w)[0] if block else np.zeros(6)
            measured_dq = array(robot.data.joint_vel)[0, arm_ids]
            settled = settled and np.max(np.abs(measured_dq)) < 0.04
            rows.append(
                {
                    "t_s": stamp,
                    "phase": phase,
                    "settled": settled,
                    "q": array(robot.data.joint_pos)[0, arm_ids],
                    "dq": measured_dq,
                    "ee_pos": ee[:3, 3].copy(),
                    "ee_quat": matrix_quat(ee[:3, :3]),
                    "target_pos": target_pos.copy(),
                    "target_quat": target_quat.copy(),
                    "wrist_raw_S": raw,
                    "wrist_raw_O_K": raw_o,
                    "wrench_O_K": raw_o.copy(),
                    "tool_com_pose": com_pose,
                    "tool_com_velocity": com_vel,
                    "tool_link_quat": bp[tool_ids, 3:].copy(),
                    "object_pose": object_pose,
                    "object_velocity": object_vel,
                    "finger_q": array(robot.data.joint_pos)[0, finger_ids],
                }
            )
            if not np.all(np.isfinite(raw_o)) or not np.all(np.isfinite(measured_dq)):
                raise RuntimeError("nonfinite wrench or joint velocity")
            if args.video and step % video_steps == 0:
                sim.render()
                rgb = annotator.get_data()
                if rgb is not None and rgb.size:
                    writer.append_data(rgb[:, :, :3])
                    frame_times.append(stamp)
            if step % max(1, round(1 / args.dt)) == 0:
                print(
                    f"[PROGRESS] sim={stamp:.3f}/{duration:.3f}s phase={phase} wrist={np.round(raw,4).tolist()} position_error={np.linalg.norm(target_pos-ee[:3,3]):.5f} wall={time.monotonic()-tic:.1f}s",
                    flush=True,
                )
                write_json(
                    args.out / "progress.json",
                    {
                        "sim_time": stamp,
                        "duration": duration,
                        "wall_seconds": time.monotonic() - tic,
                    },
                )
    finally:
        if writer:
            writer.close()
        if rows:
            np.savez_compressed(
                args.out / "raw_partial.npz",
                **{key: np.asarray([row[key] for row in rows]) for key in rows[0]},
            )
    arrays = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
    derive_frames(arrays, protocol, ee_t_k)
    tool_support = np.zeros_like(arrays["wrist_raw_O_K"])
    for i, mass in enumerate(masses):
        pose = arrays["tool_com_pose"][:, i]
        vel = arrays["tool_com_velocity"][:, i]
        acc = derivative(vel, arrays["t_s"])
        tool_support += support_wrench(
            mass,
            pose[:, :3],
            arrays["k_pos"],
            acc[:, :3],
            rotation(arrays["tool_link_quat"][:, i]),
            inertias[i],
            vel[:, 3:],
            acc[:, 3:],
        )
    arrays["tool_support_O_K"] = tool_support
    arrays["wrench_O_K"] = arrays["wrist_raw_O_K"] - tool_support
    derive_frames(arrays, protocol, ee_t_k)
    if args.empty:
        arrays["gravity_O_K"][:] = 0.0
        arrays["gravity_K_K"][:] = 0.0
        arrays["object_dynamics_O_K"] = np.zeros_like(tool_support)
    else:
        vel = arrays["object_velocity"]
        acc = derivative(vel, arrays["t_s"])
        arrays["object_dynamics_O_K"] = support_wrench(
            protocol.mass_kg,
            arrays["object_pose"][:, :3],
            arrays["k_pos"],
            acc[:, :3],
            rotation(arrays["object_pose"][:, 3:]),
            box_inertia(protocol.mass_kg, protocol.dimensions_m),
            vel[:, 3:],
            acc[:, 3:],
        )
    arrays["video_t_s"] = np.asarray(frame_times)
    object_relative = np.einsum(
        "nji,nj->ni",
        rotation(arrays["ee_quat"]),
        arrays["object_pose"][:, :3] - arrays["ee_pos"],
    )
    arrays["object_com_in_ee"] = object_relative
    residual = arrays["wrench_O_K"] - arrays["object_dynamics_O_K"]
    hold = arrays["settled"]
    hold_residual = arrays["wrench_O_K"][hold] - arrays["gravity_O_K"][hold]
    summary = {
        "samples": len(rows),
        "settled_samples": int(hold.sum()),
        "dynamic_balance_rmse": np.sqrt(np.mean(residual**2, axis=0)).tolist(),
        "static_gravity_rmse": np.sqrt(np.mean(hold_residual**2, axis=0)).tolist()
        if hold.any()
        else None,
        "max_object_com_offset_error_m": float(
            np.linalg.norm(object_relative - np.array(protocol.ee_com_m), axis=1).max()
        )
        if not args.empty
        else 0.0,
        "max_grasp_drift_m": float(
            np.linalg.norm(object_relative - object_relative[0], axis=1).max()
        )
        if not args.empty
        else 0.0,
        "max_tracking_error_m": float(
            np.linalg.norm(arrays["ee_pos"] - arrays["target_pos"], axis=1).max()
        ),
        "max_tracking_error_rad": float(
            quaternion_angle(arrays["ee_quat"], arrays["target_quat"]).max()
        ),
        "video_frames": len(frame_times),
        "elapsed_wall_seconds": time.monotonic() - tic,
    }
    quality_ok = (
        summary["max_grasp_drift_m"] <= 0.01
        and summary["max_tracking_error_m"] <= 0.04
        and summary["max_tracking_error_rad"] <= 0.4
        and (
            not args.video
            or len(frame_times) == len(range(0, total_steps, video_steps))
        )
    )
    metadata.update(
        completed=duration >= requested_duration - args.dt and quality_ok,
        quality_passed=quality_ok,
        summary=summary,
        compensation="sum of measured Newton-Euler tool-body support wrenches; payload excluded",
    )
    save_bundle(args.out, arrays, metadata)
    write_json(args.out / "summary.json", summary)
    print("[RESULT] " + json.dumps(summary), flush=True)
    if not quality_ok:
        raise RuntimeError(
            "grasp, tracking or video completeness check failed; inspect summary.json"
        )


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        import os
        import traceback

        traceback.print_exc()
        write_json(args.out / "failure.json", {"error": f"{type(exc).__name__}: {exc}"})
        # Some Kit versions exit(0) inside close(), hiding the original exception.
        # Publish the error before process teardown and preserve a failing exit code.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    simulation_app.close()
