#!/usr/bin/env python3
"""Shim: the generator now lives in ``frankatwin.excitation.chirp``.

Equivalent to the ``frankatwin-gen-chirp`` console script.
"""
import sys

from frankatwin.excitation.chirp import main

if __name__ == "__main__":
    sys.exit(main())
