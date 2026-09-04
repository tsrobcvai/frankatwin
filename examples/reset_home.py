#!/usr/bin/env python3
"""Shim: equivalent to the ``frankatwin-reset`` console script.

    frankatwin-reset [--config robot.yaml] [--speed 0.2] [--q q1 .. q7]
"""
import sys

from frankatwin.cli import reset_main

if __name__ == "__main__":
    sys.exit(reset_main())
