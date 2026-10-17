#!/usr/bin/env python3
"""Analyze a measurement bundle or compare sim and real bundles."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from frankatwin.force_gap.report import main

if __name__ == "__main__":
    main()
