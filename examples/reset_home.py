"""One-shot reset to the home joint configuration.

Run:
  python examples/reset_home.py [--config /path/to/robot.yaml] [--speed 0.2]

The blocking call internally:
  1. stops osc_shm (release libfranka session),
  2. spawns move_to --q ... (libfranka MotionGenerator, min-jerk),
  3. waits for the motion to finish,
  4. restarts osc_shm, which re-anchors its setpoint to the new EE pose.
"""

from __future__ import annotations

import argparse

import numpy as np

from panda_control.config import load_config
from panda_control.remote_client import RemotePandaClient


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=str, default=None, help="path to robot.yaml")
    p.add_argument("--speed", type=float, default=None, help="MotionGenerator speed factor (0,0.5]")
    args = p.parse_args()

    cfg = load_config(args.config)
    q_home = np.asarray(cfg.robot.init_q, dtype=np.float64)
    print(f"moving to home: {q_home}")
    with RemotePandaClient(cfg) as robot:
        robot.move_to_q(q_home, speed_factor=args.speed)
    print("done")


if __name__ == "__main__":
    main()
