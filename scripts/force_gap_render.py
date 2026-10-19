#!/usr/bin/env python3
"""Render recorded sim states with a wider camera, without rerunning dynamics.

The measurement bundle is read-only. The output contains a scene video and a
render manifest; each frame uses the original video's recorded timestamp.
"""
import argparse
import hashlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("run", type=Path)
parser.add_argument("--out", type=Path, required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
args.video = True  # Enable Isaac Lab's headless app-pump hook on sim.render().
args.out.mkdir(parents=True, exist_ok=False)
launcher = AppLauncher(args)
app = launcher.app

import imageio.v2 as imageio
import numpy as np
import torch
import carb
from pxr import Gf, Usd, UsdGeom
import omni.replicator.core as rep
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab_assets import FRANKA_PANDA_CFG
from isaaclab_physx.physics import PhysxCfg
from frankatwin.force_gap.data import load_bundle, write_json, video_times
from frankatwin.force_gap.math import pose_matrix


def main():
    data, metadata = load_bundle(args.run)
    if metadata["source"] != "sim":
        raise ValueError(
            "rendering requires recorded simulation object and finger states"
        )
    times = video_times(data, metadata)
    if len(times) < 2:
        raise ValueError("recorded video timestamps are required")
    fps = 1 / np.median(np.diff(times))
    asset = Path(metadata["robot_usd"])
    if (
        metadata.get("robot_usd_sha256")
        and hashlib.sha256(asset.read_bytes()).hexdigest()
        != metadata["robot_usd_sha256"]
    ):
        raise ValueError("robot asset changed since acquisition")
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=0.001,
            device=args.device,
            use_fabric=False,
            physics=PhysxCfg(),
            render=sim_utils.RenderCfg(
                antialiasing_mode="TAA",
                enable_dl_denoiser=False,
                enable_global_illumination=False,
                enable_reflections=False,
                carb_settings={"/rtx/rendermode": "RaytracedLighting"},
            ),
        )
    )
    ground = sim_utils.GroundPlaneCfg(color=(0.18, 0.20, 0.23))
    ground.func("/World/Ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2500.0)
    light.func("/World/Light", light)
    cfg = FRANKA_PANDA_CFG.copy()
    cfg.prim_path = "/World/Robot"
    cfg.spawn.usd_path = str(asset)
    robot = Articulation(cfg)
    block_transform = None
    if not metadata["empty"]:
        block_cfg = sim_utils.CuboidCfg(
            size=tuple(metadata["protocol"]["dimensions_m"]),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.95, 0.31, 0.06), roughness=0.65
            ),
        )
        block_cfg.func("/World/Payload", block_cfg)
        xform = UsdGeom.Xformable(sim.stage.GetPrimAtPath("/World/Payload"))
        xform.ClearXformOpOrder()
        block_transform = xform.AddTransformOp()
    camera_position = (1.10, 1.05, 0.85)
    camera_target = (0.30, 0.0, 0.40)
    camera = rep.create.camera(
        position=camera_position, look_at=camera_target, focal_length=26.0
    )
    product = rep.create.render_product(camera, (960, 720))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([product])
    sim.reset()
    settings = carb.settings.get_settings()
    for path in ("/persistent/rtx/modes/rt/enabled", "/rtx/modes/rt/enabled"):
        settings.set_bool(path, True)
    render_settings = {
        "rendermode": "RaytracedLighting",
        "sceneDb/ambientLightIntensity": 1.0,
        "domeLight/upperLowerStrategy": 3,
        "indirectDiffuse/enabled": False,
        "reflections/enabled": False,
        "translucency/enabled": False,
        "ambientOcclusion/enabled": False,
        "directLighting/sampledLighting/samplesPerPixel": 16,
    }
    for key, value in render_settings.items():
        for prefix in ("/rtx/", "/rtx-defaults/"):
            settings.set(prefix + key, value)
    arm_ids = robot.find_joints(
        [f"panda_joint{i}" for i in range(1, 8)], preserve_order=True
    )[0]
    finger_ids = robot.find_joints("panda_finger_joint.*")[0]
    link7_id = robot.body_names.index("panda_link7")
    body_transforms = []
    for name in robot.body_names:
        prim = next(
            p
            for p in Usd.PrimRange(sim.stage.GetPrimAtPath("/World/Robot"))
            if p.GetName() == name
        )
        if str(prim.GetParent().GetPath()) != "/World/Robot":
            raise ValueError("render replay requires the bundled flat-link robot asset")
        body_transforms.append(UsdGeom.Xformable(prim).MakeMatrixXform())
    link7_to_flange = np.eye(4)
    link7_to_flange[2, 3] = 0.107
    writer = imageio.get_writer(
        str(args.out / "scene.mp4"), fps=fps, codec="libx264", quality=None, bitrate="4M"
    )
    count = 0
    max_fk_error = 0.0
    try:
        for frame, stamp in enumerate(times):
            index = int(np.argmin(np.abs(data["t_s"] - stamp)))
            joint = np.zeros((1, robot.num_joints))
            joint[0, arm_ids] = data["q"][index]
            joint[0, finger_ids] = data["finger_q"][index]
            joint = torch.tensor(joint, device=args.device, dtype=torch.float32)
            robot.write_joint_state_to_sim_index(
                position=joint, velocity=torch.zeros_like(joint)
            )
            if block_transform is not None:
                pose = data["object_pose"][index]
                block_transform.Set(
                    Gf.Matrix4d(pose_matrix(pose[:3], pose[3:]).T.tolist())
                )
            sim.physics_sim_view.update_articulations_kinematic()
            robot.update(0.001)
            body_poses = robot.data.body_link_pose_w.torch[0].cpu().numpy()
            for transform, body_pose in zip(body_transforms, body_poses):
                transform.Set(
                    Gf.Matrix4d(
                        pose_matrix(body_pose[:3], body_pose[[6, 3, 4, 5]]).T.tolist()
                    )
                )
            link = body_poses[link7_id]
            ee = (
                pose_matrix(link[:3], link[[6, 3, 4, 5]])
                @ link7_to_flange
                @ np.asarray(metadata["F_T_EE"])
            )
            max_fk_error = max(
                max_fk_error, float(np.linalg.norm(ee[:3, 3] - data["ee_pos"][index]))
            )
            if max_fk_error > 0.001:
                raise RuntimeError(
                    "rendered robot FK differs from recorded state by >1 mm"
                )
            # A few render-only updates fill the temporal AA buffers, without stepping physics.
            for _ in range(8 if frame == 0 else 2):
                sim.render()
            rgb = annotator.get_data()
            if rgb is None or not rgb.size:
                raise RuntimeError("missing rendered frame")
            writer.append_data(rgb[:, :, :3])
            if frame in (0, len(times) // 2, len(times) - 1):
                imageio.imwrite(args.out / f"frame_{frame:04d}.png", rgb[:, :, :3])
            count += 1
            if frame % 100 == 0:
                print(f"[RENDER] {frame}/{len(times)}", flush=True)
    finally:
        writer.close()
    write_json(
        args.out / "render.json",
        {
            "measurement_run": str(args.run.resolve()),
            "method": "recorded-state visualization; no dynamics rerun",
            "measurement_sha256": hashlib.sha256(
                (args.run / "samples.npz").read_bytes()
            ).hexdigest(),
            "renderer": settings.get("/rtx/rendermode"),
            "render_settings": render_settings,
            "frames": count,
            "fps": fps,
            "max_recorded_ee_position_error_m": max_fk_error,
            "camera_position": camera_position,
            "camera_target": camera_target,
            "timestamps": "acquisition video_t_s or metadata video_fps cadence; nearest original state",
            "frame_times_s": times.tolist(),
            "completed": count == len(times),
        },
    )
    print(f"[RESULT] {args.out}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        import os
        import traceback

        traceback.print_exc()
        write_json(args.out / "failure.json", {"error": f"{type(exc).__name__}: {exc}"})
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    app.close()
