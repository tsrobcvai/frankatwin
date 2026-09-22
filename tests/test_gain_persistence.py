"""Gains/clamps must survive an osc_shm restart.

osc_shm re-seeds the whole shm command block on startup (new anchor + its
built-in kp 200/20, clamps off). The daemon snapshots the client's gains before
the launch and writes them back once the controller is running. This exercises
that round trip on an in-memory copy of the segment (no robot, no libfranka).
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.config import ControlConfig  # noqa: E402
from frankatwin.local_controller import (  # noqa: E402
    gains_from_config,
    restore_gains,
    snapshot_gains,
)
from frankatwin.shm_layout import SHM_TOTAL_BYTES, _build_view  # noqa: E402


def _fresh_view():
    return _build_view(memoryview(bytearray(SHM_TOTAL_BYTES)))


def _seed_like_osc_shm(view, anchor_pos, anchor_quat_wxyz):
    """Mirror the seed block at osc_shm startup: anchor + compiled-in defaults."""
    view.write_command(
        target_pos=np.asarray(anchor_pos, dtype=np.float64),
        target_quat=np.asarray(anchor_quat_wxyz, dtype=np.float64),
        kp_pos=200.0, kp_ori=20.0, kd_pos=0.0, kd_ori=0.0,
        error_delta_pos=0.0, enabled=True,
    )


def test_snapshot_is_none_on_zeroed_segment():
    assert snapshot_gains(_fresh_view()) is None


def test_gains_from_config_maps_auto_kd_to_zero():
    ctrl = ControlConfig(frequency_hz=1000, kp_pos=200.0, kp_ori=20.0,
                         kd_pos=None, kd_ori=7.5,
                         error_delta_pos=0.05)
    g = gains_from_config(ctrl)
    assert g == {"kp_pos": 200.0, "kp_ori": 20.0, "kd_pos": 0.0, "kd_ori": 7.5,
                 "error_delta_pos": 0.05, "enabled": True}


def test_restore_keeps_new_anchor_and_client_gains():
    view = _fresh_view()
    _seed_like_osc_shm(view, [0.5, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0])

    # Client raises gains / loosens clamps and disables the impedance torque.
    cur = view.read_command()
    view.write_command(
        target_pos=np.array(cur["target_pos"][0]), target_quat=np.array(cur["target_quat"][0]),
        kp_pos=500.0, kp_ori=30.0, kd_pos=0.0, kd_ori=0.0,
        error_delta_pos=0.15, enabled=False,
    )
    snap = snapshot_gains(view)
    assert snap == {"kp_pos": 500.0, "kp_ori": 30.0, "kd_pos": 0.0, "kd_ori": 0.0,
                    "error_delta_pos": 0.15, "enabled": False}

    # osc_shm restarts at a different pose and re-seeds its built-ins.
    new_pos, new_quat = [0.3, 0.1, 0.5], [0.0, 1.0, 0.0, 0.0]
    _seed_like_osc_shm(view, new_pos, new_quat)
    assert float(view.read_command()["kp_pos"][0]) == 200.0

    restore_gains(view, snap)
    cmd = view.read_command()
    np.testing.assert_allclose(cmd["target_pos"][0], new_pos)      # anchor from the restart ...
    np.testing.assert_allclose(cmd["target_quat"][0], new_quat)
    assert float(cmd["kp_pos"][0]) == 500.0                          # ... gains from the client
    assert float(cmd["kp_ori"][0]) == 30.0
    assert float(cmd["error_delta_pos"][0]) == 0.15
    assert float(cmd["error_delta_pos"][0]) == 0.15
    assert int(cmd["enabled"][0]) == 0
    assert int(cmd["seq"][0]) % 2 == 0                               # seqlock left stable
