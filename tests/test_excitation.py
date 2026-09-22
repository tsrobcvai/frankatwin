"""Shape / unit-norm / envelope sanity for the packaged excitation builders."""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.excitation import (  # noqa: E402
    CHIRP_BANDS,
    HELDOUT_FREQS,
    MULTIBAND_PROFILES,
    build_chirp_trajectory,
    build_multiband_trajectory,
    multiband_tones,
)

ANCHOR = np.array([0.5, 0.0, 0.4])
QUAT_XYZW = np.array([0.0, 0.0, 0.0, 1.0])
TOOL_DOWN_XYZW = np.array([1.0, 0.0, 0.0, 0.0])                     # 180 deg about x: the home pose


def test_multiband_shapes_and_anchor():
    t = np.arange(0.0, 12.0, 1e-3)
    x, dx, q, yaw, roll, tilt = build_multiband_trajectory(t, ANCHOR, QUAT_XYZW)
    assert x.shape == (t.size, 3) and dx.shape == (t.size, 3)
    assert q.shape == (t.size, 4) and yaw.shape == roll.shape == (t.size,) and tilt.shape == (t.size, 2)
    np.testing.assert_allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-9)
    np.testing.assert_allclose(x[0], ANCHOR, atol=1e-9)          # envelope starts at 0
    assert np.max(np.abs(x - ANCHOR)) <= 0.06 * 1.75 + 1e-9       # amp * sum of the three tones' relative amplitudes


def test_multiband_position_only_holds_orientation():
    t = np.arange(0.0, 4.0, 1e-3)
    _, _, q, yaw, roll, tilt = build_multiband_trajectory(t, ANCHOR, QUAT_XYZW, amp_yaw=0.0, amp_roll=0.0, amp_rx=0.0, amp_ry=0.0)
    assert np.all(yaw == 0.0) and np.all(roll == 0.0) and np.all(tilt == 0.0)
    np.testing.assert_allclose(np.abs(q @ QUAT_XYZW), 1.0, atol=1e-9)


def test_multiband_defaults_are_the_heldout_profile():
    t = np.arange(0.0, 12.0, 1e-3)
    default = build_multiband_trajectory(t, ANCHOR, TOOL_DOWN_XYZW)
    heldout = build_multiband_trajectory(t, ANCHOR, TOOL_DOWN_XYZW, **MULTIBAND_PROFILES["heldout"])
    for u, v in zip(default, heldout):
        np.testing.assert_array_equal(u, v)
    assert list(MULTIBAND_PROFILES) == ["heldout"]


def test_yaw_and_roll_never_tilt_a_tool_down_anchor_but_rx_ry_do():
    t = np.arange(0.0, 12.0, 1e-3)

    def tool_axis_tilt(q):                                           # angle between EE z and world -z
        return np.arccos(np.clip(2.0 * (q[:, 0] ** 2 + q[:, 1] ** 2) - 1.0, -1.0, 1.0))

    q_flat = build_multiband_trajectory(t, ANCHOR, TOOL_DOWN_XYZW, amp_yaw=0.25, amp_roll=0.20,
                                        amp_rx=0.0, amp_ry=0.0)[2]
    # yaw (base z) and roll (EE z) are the same vertical line here: q = (cos, sin, 0, 0) of (yaw - roll) / 2
    np.testing.assert_allclose(q_flat[:, 2:], 0.0, atol=1e-12)
    assert tool_axis_tilt(q_flat).max() < 1e-6

    q_ho = build_multiband_trajectory(t, ANCHOR, TOOL_DOWN_XYZW, **MULTIBAND_PROFILES["heldout"])[2]
    assert np.ptp(q_ho[:, 2]) > 0.15 and np.ptp(q_ho[:, 3]) > 0.25   # qz / qw move
    assert np.radians(15.0) < tool_axis_tilt(q_ho).max() < np.radians(25.0)


def test_tilt_amplitude_flags_scale_the_tilt():
    t = np.arange(0.0, 12.0, 1e-3)
    _, _, q, _, _, tilt = build_multiband_trajectory(t, ANCHOR, TOOL_DOWN_XYZW, amp_rx=0.1, amp_ry=0.1)
    assert 0.1 < np.abs(tilt).max() <= 0.1 * 1.75 + 1e-9
    assert np.ptp(q[:, 3]) > 0.05
    np.testing.assert_allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-9)


def test_heldout_fades_out_and_stays_inside_the_low_chirp_envelope():
    t = np.arange(0.0, 12.0 + 1e-9, 1e-3)
    x, dx, q, yaw, roll, tilt = build_multiband_trajectory(t, ANCHOR, TOOL_DOWN_XYZW, **MULTIBAND_PROFILES["heldout"])
    np.testing.assert_allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-9)
    np.testing.assert_allclose(x[0], ANCHOR, atol=1e-9)
    np.testing.assert_allclose(x[-1], ANCHOR, atol=1e-9)             # 2 s fade-out back to the anchor
    np.testing.assert_allclose(np.abs(q[-1] @ TOOL_DOWN_XYZW), 1.0, atol=1e-9)
    np.testing.assert_allclose(dx[-1], 0.0, atol=1e-9)
    assert np.all(roll == 0.0) and np.abs(tilt).max() > 0.15         # roll off, tilt on
    # the reference asks no more than the low-band chirp that has run on the arm
    tc = np.arange(0.0, 8.0 + 1e-9, 1e-3)
    xc, dxc, qc, _ = build_chirp_trajectory(tc, ANCHOR, TOOL_DOWN_XYZW, **CHIRP_BANDS["low"])

    def max_angle(quat):
        return (2.0 * np.arccos(np.clip(np.abs(quat @ TOOL_DOWN_XYZW), 0.0, 1.0))).max()

    def max_ang_speed(quat):
        return (2.0 * np.arccos(np.clip(np.abs(np.einsum("ij,ij->i", quat[1:], quat[:-1])), 0.0, 1.0)) / 1e-3).max()

    assert np.all(np.abs(x - ANCHOR).max(axis=0) <= np.abs(xc - ANCHOR).max(axis=0))
    assert np.linalg.norm(dx, axis=1).max() < np.linalg.norm(dxc, axis=1).max()
    assert max_angle(q) < max_angle(qc)
    assert max_ang_speed(q) < max_ang_speed(qc)
    assert np.linalg.norm(dx, axis=1).max() == pytest.approx(0.451, abs=5e-3)
    assert max_angle(q) == pytest.approx(0.471, abs=5e-3)


def test_heldout_tones_hold_the_reference_velocity_above_the_corner():
    tones = multiband_tones("heldout")
    freqs = [f for axis in tones.values() for f, _, _ in axis]
    assert len(freqs) == 21 == len(set(freqs))                       # 7 axes x 3 bands, all distinct
    for axis, table in tones.items():
        assert tuple(f for f, _, _ in table) == HELDOUT_FREQS[axis]
        low, mid, high = table
        assert low[1] == 1.0 and low[0] < 0.35
        assert mid[1] * mid[0] == pytest.approx(0.35) and high[1] * high[0] == pytest.approx(0.35)
    # the mid + high tones' share of the reference's position power
    share = lambda table: 1.0 - table[0][1] ** 2 / sum(a * a for _, a, _ in table)  # noqa: E731
    assert 0.11 < min(share(tones[a]) for a in ("x", "y", "z", "rx", "ry", "yaw")) < 0.12
    assert 0.32 < max(share(tones[a]) for a in ("x", "y", "z", "rx", "ry", "yaw")) < 0.33


def test_heldout_velocity_is_the_derivative_of_the_position():
    t = np.arange(0.0, 12.0 + 1e-9, 1e-3)
    x, dx = build_multiband_trajectory(t, ANCHOR, QUAT_XYZW, **MULTIBAND_PROFILES["heldout"])[:2]
    fd = np.gradient(x, 1e-3, axis=0)
    np.testing.assert_allclose(dx[5:-5], fd[5:-5], atol=1e-4)


def test_multiband_unknown_profile_raises():
    with pytest.raises(ValueError):
        build_multiband_trajectory(np.arange(0.0, 1.0, 1e-3), ANCHOR, QUAT_XYZW, profile="v9")


def test_chirp_shapes_and_ramps():
    t = np.arange(0.0, 8.0, 1e-3)
    out = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW)
    x, dx, q = out[0], out[1], out[2]
    assert x.shape == (t.size, 3) and dx.shape == (t.size, 3) and q.shape == (t.size, 4)
    np.testing.assert_allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-9)
    np.testing.assert_allclose(x[0], ANCHOR, atol=1e-6)           # 2 s ramp-up from anchor
    np.testing.assert_allclose(x[-1], ANCHOR, atol=1e-2)          # 3 s ramp-down back
    assert np.max(np.abs(x[:, 2] - ANCHOR[2])) > np.max(np.abs(x[:, 0] - ANCHOR[0]))  # z = 1.5x xy


def _accel_away_from_ramp_corners(t, dx, corners=(2.0, 5.0), margin=0.05):
    acc = np.gradient(dx, t[1] - t[0], axis=0)
    keep = (t > margin) & (t < t[-1] - margin)
    for c in corners:
        keep &= np.abs(t - c) > margin
    return acc[keep]


def test_chirp_bands_are_contiguous_and_8_s():
    lo, hi = CHIRP_BANDS["low"], CHIRP_BANDS["high"]
    assert lo["f0"] == 0.1 and lo["f1"] == 0.7 and lo["amp_taper_exp"] == 0.0
    assert (lo["ramp_up_s"], lo["ramp_down_s"]) == (2.0, 3.0)        # UR5e
    assert hi["f0"] == lo["f1"] and hi["f1"] == 3.0 and hi["amp_taper_exp"] == 0.5
    assert (hi["ramp_up_s"], hi["ramp_down_s"]) == (2.0, 1.0)        # full envelope until 7 s
    t = np.arange(0.0, 8.0 + 1e-9, 1e-3)
    for band in (lo, hi):
        x, dx, q, rot = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, **band)
        assert x.shape == (t.size, 3) and t[-1] == pytest.approx(8.0)
        np.testing.assert_allclose(x[0], ANCHOR, atol=1e-9)
        np.testing.assert_allclose(x[-1], ANCHOR, atol=1e-9)        # ramp-down back to the anchor


def test_low_band_is_the_untapered_chirp():
    t = np.arange(0.0, 8.0 + 1e-9, 1e-3)
    a = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW)                 # defaults
    b = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, **CHIRP_BANDS["low"])
    for u, v in zip(a, b):
        np.testing.assert_array_equal(u, v)
    # full amplitude inside the flat window of the 2 s / 3 s ramp
    flat = (t >= 2.0) & (t <= 5.0)
    assert np.max(np.abs(a[0][flat, 2] - ANCHOR[2])) == pytest.approx(0.15, abs=2e-3)


def test_high_band_default_tapers_as_the_square_root_and_stays_full_until_7_s():
    t = np.arange(0.0, 8.0 + 1e-9, 1e-3)
    lo = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, **CHIRP_BANDS["low"])
    hi = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, **CHIRP_BANDS["high"])

    def z_amp(t0, t1):
        w = (t >= t0) & (t <= t1)
        return np.max(np.abs(hi[0][w, 2] - ANCHOR[2]))

    f = lambda ts: 0.7 + 2.3 * ts / 8.0                              # noqa: E731  instantaneous frequency
    for ts in (2.5, 4.5, 6.5):                                       # flat envelope: 2 s .. 7 s
        assert z_amp(ts - 0.5, ts + 0.5) == pytest.approx(0.15 * (0.7 / f(ts)) ** 0.5, rel=0.12)
    assert z_amp(6.0, 7.0) > 0.07                                    # 1 s fade-out: still full at 6-7 s
    np.testing.assert_allclose(hi[0][-1], ANCHOR, atol=1e-9)
    assert np.max(np.linalg.norm(hi[3], axis=1)) < np.max(np.linalg.norm(lo[3], axis=1))


def test_exponent_1_holds_the_reference_velocity():
    t = np.arange(0.0, 8.0 + 1e-9, 1e-3)
    hi = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, f0=0.7, f1=3.0, amp_taper_exp=1.0)
    v_ref = 0.15 * 2 * np.pi * 0.7                                   # z amplitude at 0.7 Hz: 0.66 m/s
    for t0, t1 in ((2.0, 3.0), (3.0, 4.0), (4.0, 5.0)):              # flat part of the 2 s / 3 s envelope
        w = (t >= t0) & (t <= t1)
        assert np.abs(hi[1][w, 2]).max() == pytest.approx(v_ref, rel=0.05)
    # amplitude ~ 0.7/f: 49 mm on z at t = 5 s (f = 2.14 Hz), against 150 mm for the low band
    window = (t >= 4.5) & (t <= 5.0)
    assert 0.040 < np.max(np.abs(hi[0][window, 2] - ANCHOR[2])) < 0.060


def test_exponent_2_holds_the_reference_acceleration_instead():
    t = np.arange(0.0, 8.0 + 1e-9, 1e-3)
    lo = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, **CHIRP_BANDS["low"])
    hi = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, f0=0.7, f1=3.0, amp_taper_exp=2.0)
    a_ref = 0.15 * (2 * np.pi * 0.7) ** 2                            # 2.9 m/s^2
    acc_lo = np.abs(_accel_away_from_ramp_corners(t, lo[1])[:, 2]).max()
    acc_hi = np.abs(_accel_away_from_ramp_corners(t, hi[1])[:, 2]).max()
    assert acc_hi == pytest.approx(a_ref, rel=0.05)
    # the low band never gets there: its fade-out coincides with its fastest part
    assert acc_lo == pytest.approx(1.31, rel=0.05)
    window = (t >= 4.5) & (t <= 5.0)
    assert 0.010 < np.max(np.abs(hi[0][window, 2] - ANCHOR[2])) < 0.020


def test_tapered_chirp_velocity_is_the_derivative_of_the_position():
    t = np.arange(0.0, 8.0 + 1e-9, 1e-3)
    x, dx, _, _ = build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, **CHIRP_BANDS["high"])
    fd = np.gradient(x, 1e-3, axis=0)
    keep = (np.abs(t - 2.0) > 0.01) & (np.abs(t - 7.0) > 0.01) & (t > 0.01) & (t < 7.99)
    np.testing.assert_allclose(dx[keep], fd[keep], atol=4e-3)


def test_taper_needs_a_positive_f0():
    t = np.arange(0.0, 1.0, 1e-3)
    with pytest.raises(ValueError):
        build_chirp_trajectory(t, ANCHOR, QUAT_XYZW, f0=0.0, f1=1.0, amp_taper_exp=2.0)
