#!/usr/bin/env python3
"""Plot the EE reference of the multiband and chirp excitations, target only.

Builds each reference exactly as ``examples/cart_impedance.py`` does with its
default flags -- the same ``build_*_trajectory`` call on the same 50 Hz grid --
and writes two figures per mode: x/y/z position and the qx/qy/qz/qw quaternion
(xyzw). The docs figures under ``docs/images/excitation_*.png`` come from here.

The reference is anchored at the pose the arm starts from. The default anchor is
our FR3's home pose (``move_to.py`` with no arguments) with a Franka Hand, tool
pointing down; ``--sidecar`` takes the anchor from a real run's JSON sidecar
instead.

Usage (needs matplotlib):
  python scripts/plot_excitation_traj.py                       # -> docs/images/
  python scripts/plot_excitation_traj.py --sidecar data/chirp.json --out-dir /tmp/plots
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from frankatwin.excitation import build_chirp_trajectory, build_multiband_trajectory

REPO = Path(__file__).resolve().parents[1]
RATE_HZ = 50.0                                    # cart_impedance.py --rate default
DURATION_S = {"multiband": 12.0, "chirp": 8.0}    # cart_impedance.py --duration defaults
TITLE = {"multiband": "Multiband (v3)", "chirp": "Chirp (v4)"}
HOME_X = np.array([0.307, 0.000, 0.477])          # EE at home, measured on our FR3 + Hand [m]
TOOL_DOWN_XYZW = np.array([1.0, 0.0, 0.0, 0.0])


def build(mode: str, x_anchor: np.ndarray, q_anchor_xyzw: np.ndarray):
    n = int(round(DURATION_S[mode] * RATE_HZ))
    t = np.arange(n, dtype=np.float64) / RATE_HZ
    if mode == "multiband":
        x_des, _, quat_des, _, _ = build_multiband_trajectory(t, x_anchor, q_anchor_xyzw)
    else:
        x_des, _, quat_des, _ = build_chirp_trajectory(t, x_anchor, q_anchor_xyzw)
    return t, x_des, quat_des


def plot(mode: str, t, x_des, quat_des, out_dir: Path, dpi: int) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for kind, data, labels, units, figsize, title in (
        ("position", x_des, ["x", "y", "z"], " [m]", (10, 8), "EE position"),
        ("orientation", quat_des, ["qx", "qy", "qz", "qw"], "", (10, 9),
         "EE orientation quaternion (xyzw)"),
    ):
        fig, axes = plt.subplots(len(labels), 1, figsize=figsize, sharex=True)
        for idx, (ax, label) in enumerate(zip(axes, labels)):
            ax.plot(t, data[:, idx], "-", color="dimgray", linewidth=1.5)
            ax.set_ylabel(f"{label}{units}")
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("t [s]")
        fig.suptitle(f"{TITLE[mode]} target: {title}")
        fig.tight_layout()
        path = out_dir / f"excitation_{mode}_{kind}.png"
        fig.savefig(path, dpi=dpi)
        plt.close(fig)
        print(f"wrote {path}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--sidecar", type=str, default=None,
                   help="Take x_anchor / q_anchor_xyzw from this cart_impedance.py sidecar "
                        "(default: our home pose, tool down).")
    p.add_argument("--out-dir", type=str, default=str(REPO / "docs" / "images"))
    p.add_argument("--dpi", type=int, default=140)
    args = p.parse_args()

    if args.sidecar:
        side = json.loads(Path(args.sidecar).read_text(encoding="utf-8"))
        x_anchor = np.asarray(side["x_anchor"], dtype=np.float64)
        q_anchor_xyzw = np.asarray(side["q_anchor_xyzw"], dtype=np.float64)
    else:
        x_anchor, q_anchor_xyzw = HOME_X, TOOL_DOWN_XYZW

    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    for mode in ("multiband", "chirp"):
        plot(mode, *build(mode, x_anchor, q_anchor_xyzw), out_dir, args.dpi)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
