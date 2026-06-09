#!/usr/bin/env python3
"""Plot a v3/v4 excitation target CSV as 6 per-DOF panels (x,y,z,rx,ry,rz).

Position panels show the desired offset from the anchor [cm]; rotation panels
show the axis-angle offset of q_des relative to q_anchor [rad], decomposed in
the world frame.  Both versions are decoded the same way from quat_des so the
v3 and v4 figures are directly comparable.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _quat_conj_xyzw(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    out[..., :3] *= -1.0
    return out


def _quat_mul_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    w = aw * bw - ax * bx - ay * by - az * bz
    return np.stack((x, y, z, w), axis=-1)


def _quat_to_rotvec(q: np.ndarray) -> np.ndarray:
    """(T,4) xyzw unit quats -> (T,3) axis-angle vectors [rad]."""
    q = q / np.clip(np.linalg.norm(q, axis=1, keepdims=True), 1e-12, None)
    # Canonicalize to w >= 0 so theta in [0, pi].
    q = np.where(q[:, 3:4] < 0.0, -q, q)
    w = np.clip(q[:, 3], -1.0, 1.0)
    vec = q[:, :3]
    vn = np.linalg.norm(vec, axis=1)
    theta = 2.0 * np.arctan2(vn, w)
    scale = np.where(vn < 1e-12, 0.0, theta / np.where(vn < 1e-12, 1.0, vn))
    return vec * scale[:, None]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--sidecar", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="")
    args = ap.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    side = json.loads(Path(args.sidecar).read_text())
    x_anchor = np.asarray(side["x_anchor"], dtype=np.float64)
    q_anchor = np.asarray(side["q_anchor_xyzw"], dtype=np.float64)
    q_anchor = q_anchor / np.linalg.norm(q_anchor)

    t = data["t_s"]
    pos = np.column_stack((data["x_des_x"], data["x_des_y"], data["x_des_z"]))
    quat = np.column_stack(
        (data["quat_des_x"], data["quat_des_y"], data["quat_des_z"], data["quat_des_w"])
    )

    pos_off_cm = (pos - x_anchor[None, :]) * 100.0
    q_rel = _quat_mul_xyzw(quat, _quat_conj_xyzw(np.broadcast_to(q_anchor, quat.shape)))
    rot_off = _quat_to_rotvec(q_rel)  # rad, world frame

    pos_labels = ["x", "y", "z"]
    rot_labels = ["rx", "ry", "rz"]
    pos_colors = ["#1f77b4", "#2ca02c", "#d62728"]
    rot_colors = ["#9467bd", "#8c564b", "#e377c2"]

    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    for r in range(3):
        ax_p = axes[r, 0]
        ax_p.plot(t, pos_off_cm[:, r], color=pos_colors[r], lw=1.1)
        ax_p.axhline(0.0, color="0.7", lw=0.6, zorder=0)
        peak = np.max(np.abs(pos_off_cm[:, r]))
        ax_p.set_ylabel(f"Δ{pos_labels[r]} [cm]")
        ax_p.set_title(f"{pos_labels[r]}   peak |Δ| = {peak:.2f} cm", fontsize=10, loc="left")
        ax_p.grid(alpha=0.25)

        ax_r = axes[r, 1]
        ax_r.plot(t, rot_off[:, r], color=rot_colors[r], lw=1.1)
        ax_r.axhline(0.0, color="0.7", lw=0.6, zorder=0)
        peak_r = np.max(np.abs(rot_off[:, r]))
        ax_r.set_ylabel(f"Δ{rot_labels[r]} [rad]")
        ax_r.set_title(
            f"{rot_labels[r]}   peak |Δ| = {peak_r:.3f} rad ({np.degrees(peak_r):.1f}°)",
            fontsize=10,
            loc="left",
        )
        ax_r.grid(alpha=0.25)

    axes[2, 0].set_xlabel("t [s]")
    axes[2, 1].set_xlabel("t [s]")
    fig.suptitle(args.title, fontsize=13, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print(f"[plot] wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
