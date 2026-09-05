#!/usr/bin/env python3
"""Open / close / home / stop the Franka Hand, or read its state.

Run:
  python examples/gripper.py --open                        # fully open (gripper.max_width = 0.08 m)
  python examples/gripper.py --open --width 0.42           # open to 42 % of the stroke (33.6 mm)
  python examples/gripper.py --open --width-m 0.03         # open to 30 mm
  python examples/gripper.py --close                       # grasp: squeeze at gripper.grasp_force (70 N)
  python examples/gripper.py --close --force 30            # gentler hold
  python examples/gripper.py --close --close-width 0.008   # stop the jaws at 8 mm instead of squeezing past
  python examples/gripper.py --homing                      # once after power-up / a finger change
  python examples/gripper.py --stop                        # abort a motion in flight
  python examples/gripper.py --state                       # width / max_width / is_grasped / temperature
  python examples/gripper.py ... --config /path/to/robot.yaml

The arm controller keeps running throughout: the hand is served on its own port
(1338), so nothing here stops or restarts osc_shm.

CLOSING IS A `grasp`, NOT A WIDTH COMMAND. libfranka's Gripper::grasp(width, speed,
force, eps) drives the fingers TOWARDS width and squeezes with force once they
stall on something. The default width (-0.01, past full closure) always reaches
the object, so the resting width is decided by the OBJECT, not by the command:

  * to change how HARD it holds, use --force (default gripper.grasp_force = 70 N,
    the Franka Hand's rated continuous maximum; pass less for anything crushable);
  * --close-width only stops the jaws EARLY -- set it above the object's width and
    they halt before touching, nothing is held;
  * --eps narrows the success band around --close-width that `result` reports.
    The default 0.08 (whole stroke) makes every stall count as grasped.

--width is a fraction of the stroke (0-1, as in the deoxys-based
control_gripper.py this replaces); --width-m is metres.
"""

from __future__ import annotations

import argparse
import json

from frankatwin.config import load_config
from frankatwin.remote_client import FrankaTwinClient


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--open", action="store_true", help="open (Gripper::move) to --width / --width-m")
    g.add_argument("--close", action="store_true", help="close (Gripper::grasp) with --force")
    g.add_argument("--homing", action="store_true", help="calibrate the stroke (once after power-up)")
    g.add_argument("--stop", action="store_true", help="abort the motion in flight")
    g.add_argument("--state", action="store_true", help="print width / max_width / is_grasped / temperature")

    w = p.add_mutually_exclusive_group()
    w.add_argument("--width", type=float, default=None, metavar="FRAC",
                   help="open: fraction of gripper.max_width in [0, 1] (default 1.0 = fully open)")
    w.add_argument("--width-m", type=float, default=None, metavar="M",
                   help="open: width in metres, [0, gripper.max_width]")
    p.add_argument("--speed", type=float, default=None, metavar="M_S",
                   help="finger speed [m/s]; default gripper.move_speed (open) / gripper.grasp_speed (close)")
    p.add_argument("--force", type=float, default=None, metavar="N",
                   help="close: grasp force in (0, 70] N; default gripper.grasp_force")
    p.add_argument("--close-width", type=float, default=None, metavar="M",
                   help="close: target width the fingers drive towards; default gripper.grasp_width (-0.01)")
    p.add_argument("--eps", type=float, default=None, metavar="M",
                   help="close: epsilon_inner = epsilon_outer success band; default gripper.epsilon_*")
    p.add_argument("--no-wait", action="store_true", help="return as soon as the daemon accepted the command")
    p.add_argument("--config", type=str, default=None, help="path to robot.yaml")
    args = p.parse_args()

    cfg = load_config(args.config)
    wait = not args.no_wait
    with FrankaTwinClient(cfg) as robot:
        if args.open:
            if args.width_m is not None:
                width = args.width_m
            else:
                frac = 1.0 if args.width is None else args.width
                if not (0.0 <= frac <= 1.0):
                    p.error("--width is a fraction of the stroke in [0, 1]; use --width-m for metres")
                width = frac * cfg.gripper.max_width
            print(f"opening to {width * 1e3:.1f} mm ...")
            out = robot.gripper_open(width, args.speed, wait=wait)
        elif args.close:
            eps = {} if args.eps is None else {"epsilon_inner": args.eps, "epsilon_outer": args.eps}
            force = cfg.gripper.grasp_force if args.force is None else args.force
            print(f"closing: grasp towards {cfg.gripper.grasp_width if args.close_width is None else args.close_width:+.3f} m "
                  f"at {force:g} N ...")
            out = robot.gripper_close(width=args.close_width, speed=args.speed, force=args.force, wait=wait, **eps)
        elif args.homing:
            print("homing (sweeps the full stroke) ...")
            out = robot.gripper_homing(wait=wait)
        elif args.stop:
            out = robot.gripper_stop()
        else:
            out = {"state": robot.gripper_state()}

        st = out.get("state")
        if st is not None:
            print(f"width = {st['width'] * 1e3:.1f} mm  (max {st['max_width'] * 1e3:.1f} mm)  "
                  f"is_grasped = {st['is_grasped']}  temperature = {st['temperature']} C")
        if "result" in out and not args.state:
            print(f"result = {out['result']}" + ("  (stopped)" if out.get("stopped") else ""))
        if args.no_wait:
            print(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
