#!/usr/bin/env python3
"""Shim: the excitation runner now lives in ``frankatwin.tools.cart_impedance``.

Equivalent to the ``frankatwin-excite`` console script; run
``frankatwin-excite --help`` for the modes (sine / multiband / chirp).
"""
import sys

from frankatwin.tools.cart_impedance import main

if __name__ == "__main__":
    sys.exit(main())
