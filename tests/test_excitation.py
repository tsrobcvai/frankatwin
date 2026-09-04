"""Shape / unit-norm / envelope sanity for the packaged excitation builders."""

from __future__ import annotations

import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.excitation import (  # noqa: E402
    build_chirp_trajectory,
    build_multiband_trajectory,
)

ANCHOR = np.array([0.5, 0.0, 0.4])
QUAT_XYZW = np.array([0.0, 0.0, 0.0, 1.0])


def test_multiband_shapes_and_anchor():
    t = np.arange(0.0, 12.0, 1e-3)
    x, dx, q, yaw, roll = build_multiband_trajectory(t, ANCHOR, QUAT_XYZW)
    assert x.shape == (t.size, 3) and dx.shape == (t.size, 3)
    assert q.shape == (t.size, 4) and yaw.shape == roll.shape == (t.size,)
    np.testing.assert_allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-9)
    np.testing.assert_allclose(x[0], ANCHOR, atol=1e-9)          # envelope starts at 0
    assert np.max(np.abs(x - ANCHOR)) <= 0.10 * 1.2 + 1e-9        # amp * (1 + high band)


def test_multiband_position_only_holds_orientation():
    t = np.arange(0.0, 4.0, 1e-3)
    _, _, q, yaw, roll = build_multiband_trajectory(t, ANCHOR, QUAT_XYZW, amp_yaw=0.0, amp_roll=0.0)
    assert np.all(yaw == 0.0) and np.all(roll == 0.0)
    np.testing.assert_allclose(np.abs(q @ QUAT_XYZW), 1.0, atol=1e-9)


def test_chirp_shapes_and_ramps():
    t = np.arange(0.0, 8.0, 1e-3)
    out = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW)
    x, dx, q = out[0], out[1], out[2]
    assert x.shape == (t.size, 3) and dx.shape == (t.size, 3) and q.shape == (t.size, 4)
    np.testing.assert_allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-9)
    np.testing.assert_allclose(x[0], ANCHOR, atol=1e-6)           # 2 s ramp-up from anchor
    np.testing.assert_allclose(x[-1], ANCHOR, atol=1e-2)          # 3 s ramp-down back
    assert np.max(np.abs(x[:, 2] - ANCHOR[2])) > np.max(np.abs(x[:, 0] - ANCHOR[0]))  # z = 1.5x xy
