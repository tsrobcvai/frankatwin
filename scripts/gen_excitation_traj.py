#!/usr/bin/env python3
"""Shim: the generator now lives in ``frankatwin.excitation.multiband``.

Equivalent to the ``frankatwin-gen-multiband`` console script.
"""
import sys

from frankatwin.excitation.multiband import main

if __name__ == "__main__":
    sys.exit(main())
