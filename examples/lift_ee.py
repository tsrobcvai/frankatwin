"""Lift the end-effector straight up by a small height from its current pose.

Run:
  python examples/lift_ee.py                          # +1 cm over 2 s, orientation held
  python examples/lift_ee.py --height 0.02 --duration 3.0
  python examples/lift_ee.py --config /path/to/robot.yaml

What it does:
  1. reads the current EE pose from the osc_shm state stream,
  2. ramps the Cartesian z-target from the current height up by --height metres
     over --duration seconds (smoothstep ramp -> zero velocity at both ends, so
     osc_shm never sees a step larger than its per-tick error_delta_pos clamp),
     holding orientation fixed the whole time,
  3. holds the lifted pose briefly and reports the achieved z delta.

This is task-impedance control (osc_shm), NOT a joint-space move: +z is "up" in
the robot base frame. The arm is compliant, so the final height tracks the
target within the impedance stiffness (a light payload sags a few mm).

Prerequisites (see README):
  * daemon running on the NUC:  python -m panda_control.daemon -c config/robot.yaml
    (the daemon launches osc_shm, which this script drives).
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from panda_control.config import load_config
from panda_control.remote_client import RemotePandaClient


def main() -> None:
    p = argparse.ArgumentParser(description="Lift the EE straight up by a fixed height.")
    p.add_argument("--config", type=str, default=None, help="path to robot.yaml")
    p.add_argument("--height", type=float, default=0.01, help="lift height in metres (default: 0.01 = 1 cm)")
    p.add_argument("--duration", type=float, default=2.0, help="ramp duration in seconds (default: 2.0)")
    p.add_argument("--rate", type=float, default=50.0, help="command rate in Hz (default: 50)")
    p.add_argument("--hold", type=float, default=0.5, help="seconds to hold the lifted pose before readback")
    args = p.parse_args()

    cfg = load_config(args.config)
    dt = 1.0 / args.rate
    n_steps = max(1, int(round(args.duration / dt)))

    with RemotePandaClient(cfg) as robot:
        state = robot.wait_for_state(timeout_s=3.0)
        anchor_pos = state.ee_pos.astype(np.float64).copy()
        anchor_quat = state.ee_quat.astype(np.float64).copy()  # wxyz, held fixed
        print(f"current  ee_pos = {anchor_pos}")
        print(f"lifting  +{args.height * 100:.1f} cm in z over {args.duration:.1f} s")

        target = anchor_pos.copy()
        for k in range(1, n_steps + 1):
            s = k / n_steps
            smooth = s * s * (3.0 - 2.0 * s)  # smoothstep: 0 at s=0, 1 at s=1
            target[2] = anchor_pos[2] + args.height * smooth
            robot.set_ee_target(target, anchor_quat)
            time.sleep(dt)

        time.sleep(args.hold)  # let the impedance controller settle at the top

        final = robot.get_state().ee_pos
        achieved = final[2] - anchor_pos[2]
        print(f"final    ee_pos = {final}")
        print(f"achieved z delta = {achieved * 1000:.1f} mm (target {args.height * 1000:.1f} mm)")


if __name__ == "__main__":
    main()
