#!/usr/bin/env python3
"""A policy running at a fixed rate on top of the 1 kHz task-impedance controller.

Every closed-loop rollout has this shape: each policy step reads the robot
state, runs the policy, turns the action into an EE pose target and sends it;
osc_shm holds that target at 1 kHz until the next step (zero-order hold -- the
same staircase the IsaacLab replay reproduces).

Run (daemon running on the NUC):
  python examples/policy_loop.py                          # demo policy: 10 cm up / down every 4 s, for 16 s
  python examples/policy_loop.py --hz 20 --pos-scale 0.0025
  python examples/policy_loop.py --kp-pos 500 --kp-ori 30 --no-reset

Action convention (6-D, each component in [-1, 1]):
  a[0:3]  Δposition = pos_scale * a[0:3]  [m/step], robot base frame
  a[3:6]  Δrotation = rot_scale * a[3:6]  [rad/step], axis-angle in the base frame:
          q_target = q(Δ) ⊗ q_current
Deltas are applied to the *measured* EE pose (as the IsaacLab tasks do:
ctrl_target = fingertip_pos + action). Replace `demo_policy` with your policy
and build `obs` to match its training layout.
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from frankatwin.config import load_config
from frankatwin.quat import from_rotvec_wxyz, mul_wxyz
from frankatwin.remote_client import FrankaTwinClient


def demo_policy(t: float, obs: np.ndarray) -> np.ndarray:
    """Stand-in for a network: up for 2 s, down for 2 s, repeat.

    At 10 Hz with pos_scale = 0.005 m/step that is a 10 cm stroke every 4 s.
    """
    a = np.zeros(6)
    a[2] = 1.0 if t % 4.0 < 2.0 else -1.0
    return a


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--hz", type=float, default=10.0, help="policy rate [Hz]")
    p.add_argument("--duration", type=float, default=16.0, help="run time [s]")
    p.add_argument("--pos-scale", type=float, default=0.005, help="|Δpos| per unit action [m/step]")
    p.add_argument("--rot-scale", type=float, default=0.02, help="|Δrot| per unit action [rad/step]")
    p.add_argument("--kp-pos", type=float, default=500.0)
    p.add_argument("--kp-ori", type=float, default=30.0)
    p.add_argument("--err-delta-pos", type=float, default=0.0,
                   help="osc_shm error clamp [m]; 0 (default) = pure impedance, as in sim. Above one step it never engages but still catches runaway tracking")
    p.add_argument("--no-reset", action="store_true", help="skip the initial move_to home")
    p.add_argument("--config", type=str, default=None, help="path to robot.yaml")
    args = p.parse_args()

    cfg = load_config(args.config)
    dt = 1.0 / args.hz
    with FrankaTwinClient(cfg) as robot:
        if not args.no_reset:
            robot.move_to_q(cfg.robot.init_q)                       # position control: blocking reset
        robot.set_gains(kp_pos=args.kp_pos, kp_ori=args.kp_ori,     # impedance gains, Kd = 2*sqrt(Kp)
                        error_delta_pos=args.err_delta_pos)
        s = robot.wait_for_state()
        print(f"[policy_loop] {args.hz:g} Hz for {args.duration:g} s; start ee_pos = {np.round(s.ee_pos, 4).tolist()}")

        t0 = time.monotonic()
        for k in range(int(args.duration * args.hz)):
            t = k * dt
            obs = np.concatenate([s.q, s.dq, s.ee_pos, s.ee_quat, s.ee_linvel, s.ee_angvel])
            a = np.clip(demo_policy(t, obs), -1.0, 1.0)

            pos = s.ee_pos + args.pos_scale * a[:3]
            quat = mul_wxyz(from_rotvec_wxyz(args.rot_scale * a[3:]), s.ee_quat)
            robot.set_ee_target(pos, quat)                          # non-blocking; held at 1 kHz until next step

            time.sleep(max(0.0, t0 + (k + 1) * dt - time.monotonic()))   # fixed-rate tick, no drift
            s = robot.get_state() or s                              # newest frame of the 100 Hz stream

        robot.set_ee_target(s.ee_pos, s.ee_quat)                    # hold where we are
        print(f"[policy_loop] done; end ee_pos = {np.round(s.ee_pos, 4).tolist()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
