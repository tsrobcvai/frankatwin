#!/usr/bin/env python3
"""Shim: equivalent to the ``frankatwin-reset`` console script.

    frankatwin-reset                                   # joint-space move to robot.init_q
    frankatwin-reset --q q1 .. q7 [--speed 0.2]        # joint-space move to any configuration
    frankatwin-reset --pose x y z qw qx qy qz [--duration 5]   # EE-pose move (position control)
"""
import sys

from frankatwin.cli import reset_main

if __name__ == "__main__":
    sys.exit(reset_main())
