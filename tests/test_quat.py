"""Quaternion helpers: rotvec round-trip, multiply/conjugate, small-angle error, wxyz<->xyzw."""

import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.quat import (  # noqa: E402
    conj_wxyz, error_rotvec_wxyz, from_rotvec_wxyz, mul_wxyz, to_rotvec_wxyz, wxyz_to_xyzw, xyzw_to_wxyz,
)

I = np.array([1.0, 0.0, 0.0, 0.0])


def test_rotvec_roundtrip_and_known_values():
    np.testing.assert_allclose(from_rotvec_wxyz([np.pi, 0, 0]), [0, 1, 0, 0], atol=1e-12)   # tool-down
    np.testing.assert_allclose(from_rotvec_wxyz([0, 0, 0]), I)
    r = np.array([0.1, -0.2, 0.3])
    np.testing.assert_allclose(to_rotvec_wxyz(from_rotvec_wxyz(r)), r, atol=1e-12)


def test_mul_conj_identity():
    q = from_rotvec_wxyz([0.4, 0.5, -0.6])
    np.testing.assert_allclose(mul_wxyz(q, conj_wxyz(q)), I, atol=1e-12)
    np.testing.assert_allclose(mul_wxyz(I, q), q)


def test_error_matches_small_delta():
    q_cur = from_rotvec_wxyz([0.2, 0.0, 0.1])
    d = np.array([0.01, -0.02, 0.005])
    q_des = mul_wxyz(from_rotvec_wxyz(d), q_cur)          # world-frame delta on top of current
    np.testing.assert_allclose(error_rotvec_wxyz(q_des, q_cur), d, atol=1e-6)
    np.testing.assert_allclose(error_rotvec_wxyz(q_cur, q_cur), 0.0, atol=1e-12)
    np.testing.assert_allclose(error_rotvec_wxyz(-q_des, q_cur), d, atol=1e-6)   # sign-flip invariant


def test_order_conversions():
    q = np.array([0.1, 0.2, 0.3, 0.4])
    np.testing.assert_allclose(xyzw_to_wxyz(wxyz_to_xyzw(q)), q)
