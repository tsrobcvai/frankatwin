"""One-shot move to an arbitrary joint configuration, with optional readback.

Run:
  python examples/move_to_q.py --q -0.1826 0.1244 0.1174 -2.3839 0.0065 1.9799 0.7087
  python examples/move_to_q.py --q ... --speed 0.1 --config /path/to/robot.yaml

The blocking move_to_q call internally:
  1. stops osc_shm (release libfranka session),
  2. spawns move_to --q ... (libfranka MotionGenerator, min-jerk),
  3. waits for the motion to finish,
  4. restarts osc_shm, which re-anchors its setpoint to the new EE pose.

move_to (libfranka joint position control) lands on the goal accurately. But
step 4 hands control to osc_shm, a *Cartesian torque-impedance* controller that
relies on the load/gravity model. With the mounted camera payload under-modeled,
gravity sags the most-loaded joints (J2 shoulder, J6 wrist) a couple degrees
while the rest stay <0.2 deg. --verify reads the held pose back so you can see
this residual. It is NOT a move_to error and cannot be fixed by re-commanding
joints (that fights gravity and trips cartesian_reflex). To *hold* an exact
joint config, hold in joint space instead of handing off to osc_shm -- run the
move_to binary directly on the NUC (see README) so the robot's idle controller
keeps the pose, or fix the osc_shm load model.

--q takes 7 absolute joint angles in radians (J1..J7), not deltas.
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from frankatwin.config import load_config
from frankatwin.remote_client import FrankaTwinClient

RAD2DEG = 180.0 / np.pi


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--q",
        type=float,
        nargs=7,
        required=True,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="7 absolute joint angles in radians",
    )
    p.add_argument("--config", type=str, default=None, help="path to robot.yaml")
    p.add_argument("--speed", type=float, default=None, help="MotionGenerator speed factor (0,0.5]")
    p.add_argument(
        "--verify",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="read the osc_shm held pose back and report per-joint residual (default: on)",
    )
    p.add_argument("--settle", type=float, default=0.5, help="seconds to let osc_shm settle before readback")
    args = p.parse_args()

    cfg = load_config(args.config)
    q_target = np.asarray(args.q, dtype=np.float64)

    with FrankaTwinClient(cfg) as robot:
        print(f"moving to target: {q_target}")
        robot.move_to_q(q_target, speed_factor=args.speed)

        if not args.verify:
            print("done (no verify)")
            return

        if args.settle > 0.0:
            time.sleep(args.settle)
        state = robot.get_state(fresh=True)
        if state is None:
            print("done (no state frame from daemon to verify)")
            return

        q_held = np.asarray(state.q, dtype=np.float64).reshape(-1)
        err_deg = (q_held - q_target) * RAD2DEG
        print("  joint   target[rad]   held[rad]     Δ[deg]")
        for j in range(7):
            print(f"  J{j + 1}     {q_target[j]:+9.4f}    {q_held[j]:+9.4f}    {err_deg[j]:+7.3f}")
        max_dev = float(np.max(np.abs(err_deg)))
        worst = int(np.argmax(np.abs(err_deg))) + 1
        print(f"  max |Δ| = {max_dev:.3f} deg @ J{worst}  (osc_shm hold; sag = load-model residual)")


if __name__ == "__main__":
    main()
