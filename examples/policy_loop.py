#!/usr/bin/env python3
"""A policy running at a fixed rate on top of the 1 kHz task-impedance controller.

This is the pattern every closed-loop rollout uses: at each policy step read the
robot state, run the policy, turn its action into an EE pose target, send it,
wait for the next tick. osc_shm holds the target (zero-order hold) at 1 kHz in
between -- exactly what the IsaacLab replay reproduces, so what you see here is
what the sim saw.

Run (daemon running on the NUC):
  python examples/policy_loop.py                       # demo policy, 10 Hz, 20 s
  python examples/policy_loop.py --hz 20 --steps 400 --pos-scale 0.01 --rot-scale 0.02
  python examples/policy_loop.py --kp-pos 500 --kp-ori 30

Action convention (6-D, each component in [-1, 1]):
  a[0:3]  Δposition, scaled by --pos-scale [m/step], in the robot base frame
  a[3:6]  Δrotation as an axis-angle vector, scaled by --rot-scale [rad/step],
          applied in the base (world) frame: q_target = q(Δ) ⊗ q_current
The deltas are applied to the *measured* EE pose, as in the IsaacLab tasks
(ctrl_target = fingertip_pos + action), so the policy sees its own tracking
error. Apply them to the previous target instead if your sim does that.

Swap `DemoPolicy` for your network. The observation vector below is the
21-D proprio layout (q, dq, ee_pos, ee_quat wxyz, ee_linvel, ee_angvel) --
change it to match your policy's training data.
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from frankatwin.config import load_config
from frankatwin.quat import from_rotvec_wxyz, mul_wxyz
from frankatwin.remote_client import FrankaTwinClient, RobotState


class DemoPolicy:
    """Stands in for a network: slow sinusoid in z, nothing else."""

    def __init__(self, hz: float, period_s: float = 6.0):
        self.dt, self.period, self.k = 1.0 / hz, period_s, 0

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        self.k += 1
        a = np.zeros(6)
        a[2] = np.cos(2 * np.pi * self.k * self.dt / self.period)  # Δz ∈ [-1, 1]
        return a


def observation(s: RobotState) -> np.ndarray:
    return np.concatenate([s.q, s.dq, s.ee_pos, s.ee_quat, s.ee_linvel, s.ee_angvel])


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--hz", type=float, default=10.0, help="policy rate [Hz]")
    p.add_argument("--steps", type=int, default=200, help="number of policy steps")
    p.add_argument("--pos-scale", type=float, default=0.02, help="max |Δpos| per step [m]")
    p.add_argument("--rot-scale", type=float, default=0.05, help="max |Δrot| per step [rad]")
    p.add_argument("--kp-pos", type=float, default=500.0)
    p.add_argument("--kp-ori", type=float, default=30.0)
    p.add_argument("--err-delta-pos", type=float, default=0.15,
                   help="osc_shm error clamp [m]; keep it > pos-scale so it never engages (sim has no clip)")
    p.add_argument("--err-delta-rot", type=float, default=0.80)
    p.add_argument("--no-reset", action="store_true", help="skip the initial move_to home")
    p.add_argument("--config", type=str, default=None, help="path to robot.yaml")
    args = p.parse_args()

    cfg = load_config(args.config)
    policy = DemoPolicy(args.hz)
    dt = 1.0 / args.hz

    with FrankaTwinClient(cfg) as robot:
        if not args.no_reset:
            robot.move_to_q(cfg.robot.init_q)                     # position control, blocking
        robot.set_gains(kp_pos=args.kp_pos, kp_ori=args.kp_ori,   # impedance gains, Kd = 2*sqrt(Kp)
                        error_delta_pos=args.err_delta_pos, error_delta_rot=args.err_delta_rot)
        s = robot.wait_for_state()
        p0 = s.ee_pos.copy()
        print(f"[policy_loop] {args.hz:g} Hz x {args.steps} steps; start ee_pos = {np.round(p0, 4).tolist()}")

        late = 0
        t_next = time.monotonic()
        for step in range(args.steps):
            obs = observation(s)
            a = np.clip(policy(obs), -1.0, 1.0)

            pos = s.ee_pos + args.pos_scale * a[:3]
            quat = mul_wxyz(from_rotvec_wxyz(args.rot_scale * a[3:]), s.ee_quat)
            robot.set_ee_target(pos, quat)                        # non-blocking; held at 1 kHz until next step

            t_next += dt
            slack = t_next - time.monotonic()
            if slack < 0:
                late += 1
            else:
                time.sleep(slack)
            s = robot.get_state() or s                            # newest frame of the 100 Hz stream

        robot.set_ee_target(s.ee_pos, s.ee_quat)                  # hold where we are
        print(f"[policy_loop] done; end ee_pos = {np.round(s.ee_pos, 4).tolist()}; "
              f"late steps: {late}/{args.steps}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
