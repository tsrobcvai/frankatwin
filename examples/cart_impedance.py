"""Cartesian task-impedance example: small z-axis sinusoid at 20 Hz.

Prerequisites:
  - On the NUC: `python -m panda_control.daemon --config config/robot.yaml`
  - On this machine: `pip install -e .` (or PYTHONPATH=python)

Run:
  python examples/cart_impedance.py [--amp 0.05] [--freq 0.5] [--duration 4]
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from panda_control.config import load_config
from panda_control.remote_client import RemotePandaClient


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--amp", type=float, default=0.05, help="z-axis amplitude (m)")
    p.add_argument("--freq", type=float, default=0.5, help="sinusoid frequency (Hz)")
    p.add_argument("--duration", type=float, default=4.0, help="total run time (s)")
    p.add_argument("--rate", type=float, default=20.0, help="setpoint update rate (Hz)")
    p.add_argument("--config", type=str, default=None, help="path to robot.yaml")
    args = p.parse_args()

    cfg = load_config(args.config)
    with RemotePandaClient(cfg) as robot:
        # Push the same gains the daemon already seeded; explicit for clarity.
        robot.set_gains(kp_pos=cfg.control.kp_pos, kp_ori=cfg.control.kp_ori)

        state = robot.wait_for_state(timeout_s=3.0)
        anchor_pos = state.ee_pos.copy()
        anchor_quat = state.ee_quat.copy()
        print(f"anchor_pos = {anchor_pos}")
        print(f"anchor_quat (wxyz) = {anchor_quat}")

        dt = 1.0 / args.rate
        n = int(round(args.duration * args.rate))
        omega = 2.0 * np.pi * args.freq
        t0 = time.monotonic()
        for i in range(n):
            t = i * dt
            target = anchor_pos + np.array([0.0, 0.0, args.amp * np.sin(omega * t)])
            robot.set_ee_target(target, anchor_quat)
            # Sleep to the next slot.
            sleep_for = (t0 + (i + 1) * dt) - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)

        # Return to anchor and let the OSC settle.
        robot.set_ee_target(anchor_pos, anchor_quat)
        time.sleep(0.5)
        final = robot.get_state()
        if final is not None:
            err = np.linalg.norm(final.ee_pos - anchor_pos)
            print(f"final |ee_pos - anchor| = {err:.4f} m")


if __name__ == "__main__":
    main()
