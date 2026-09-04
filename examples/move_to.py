#!/usr/bin/env python3
"""Position-controlled move to a target: joint configuration, EE pose, or home.

Run:
  python examples/move_to.py                                        # home = robot.init_q (config/robot.yaml)
  python examples/move_to.py --target-joints 0 -0.785 0 -2.356 0 1.571 0.785 [--speed 0.2]
  python examples/move_to.py --target-ee 0.4 0.0 0.3  0 1 0 0 [--duration 5]
  python examples/move_to.py ... --config /path/to/robot.yaml

Target formats
  --target-joints J1 .. J7   7 absolute joint angles [rad] (not deltas).
  --target-ee x y z qw qx qy qz
      x y z        EE position [m] in the robot base frame ("O" frame, the one
                   libfranka reports as O_T_EE; +x forward, +z up).
      qw qx qy qz  EE orientation as a unit quaternion in **wxyz** order --
                   the same convention as RobotState.ee_quat / set_ee_target.
                   0 1 0 0 = 180 deg about x = tool pointing straight down
                   (the Franka "ready" orientation).
      The EE frame is libfranka's end-effector frame as configured in Desk
      (for a Franka Hand: the TCP between the fingertips). Read a valid pose
      off the robot first: `python -c "from frankatwin import *; ..."` or
      simply look at the ee_pos / ee_quat this script prints after a move.

What happens (both modes are blocking, done by the C++ `move_to` binary):
  1. the daemon stops osc_shm (libfranka allows one session at a time),
  2. joints: libfranka MotionGenerator, min-jerk, speed scaled by --speed
     EE pose: libfranka CartesianPose, 5th-order profile over --duration
     (libfranka solves the IK internally; no IK on our side),
  3. osc_shm restarts anchored at the new pose, gains/clamps preserved.

Readback: after the move the script prints the pose osc_shm is now holding.
osc_shm is a Cartesian torque-impedance controller that relies on the
gravity/load model, so with an under-modelled payload the most-loaded joints
(J2 shoulder, J6 wrist) can sag a degree or two relative to the move_to goal
while the rest stay < 0.2 deg. That is a load-model residual, not a move_to
error (see docs/troubleshooting.md, "Payload compensation").
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from frankatwin.config import load_config
from frankatwin.remote_client import FrankaTwinClient

RAD2DEG = 180.0 / np.pi


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    g = p.add_mutually_exclusive_group()
    g.add_argument("--target-joints", type=float, nargs=7,
                   metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
                   help="7 absolute joint angles [rad]; default: robot.init_q (home)")
    g.add_argument("--target-ee", type=float, nargs=7,
                   metavar=("X", "Y", "Z", "QW", "QX", "QY", "QZ"),
                   help="EE pose: position [m] in the base frame + unit quaternion wxyz")
    p.add_argument("--speed", type=float, default=None,
                   help="joint move: MotionGenerator speed factor (0, 0.5]; default reset.joint_speed_factor")
    p.add_argument("--duration", type=float, default=None,
                   help="EE move: seconds in [1.5, 20]; default reset.pose_duration")
    p.add_argument("--config", type=str, default=None, help="path to robot.yaml")
    p.add_argument("--settle", type=float, default=0.5, help="seconds to let osc_shm settle before readback")
    args = p.parse_args()

    cfg = load_config(args.config)
    with FrankaTwinClient(cfg) as robot:
        if args.target_ee is not None:
            pos = np.asarray(args.target_ee[:3], dtype=np.float64)
            quat = np.asarray(args.target_ee[3:], dtype=np.float64)
            print(f"moving EE to pos = {pos.tolist()}  quat(wxyz) = {quat.tolist()}")
            robot.move_to_pose(pos, quat, duration=args.duration)
            q_target = None
        else:
            q_target = np.asarray(
                args.target_joints if args.target_joints is not None else cfg.robot.init_q,
                dtype=np.float64,
            )
            print(f"moving to {'joints' if args.target_joints is not None else 'home'}: {np.round(q_target, 4).tolist()}")
            robot.move_to_q(q_target, speed_factor=args.speed)

        # Readback of the pose osc_shm is now holding.
        time.sleep(max(args.settle, 0.0))
        s = robot.get_state(fresh=True)
        if s is None:
            print("done (no state frame from daemon to verify)")
            return 0
        print(f"held: ee_pos = {np.round(s.ee_pos, 4).tolist()}  ee_quat(wxyz) = {np.round(s.ee_quat, 4).tolist()}")
        print(f"      q = {np.round(s.q, 4).tolist()}")
        if q_target is not None:
            err_deg = (np.asarray(s.q) - q_target) * RAD2DEG
            worst = int(np.argmax(np.abs(err_deg))) + 1
            print(f"      max |q_held - q_target| = {np.max(np.abs(err_deg)):.3f} deg @ J{worst}"
                  "  (osc_shm hold; sag = load-model residual)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
