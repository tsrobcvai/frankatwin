"""The command-block writers must survive a read-modify-write round trip.

`ShmView.read_command()` returns a shape-(1,) structured array, because the
block is mapped with `np.frombuffer(..., count=1)`. Every field access
therefore needs a `[0]`: `float(cmd["kp_pos"])` on a shape-(1,) array raises
`TypeError: only 0-dimensional arrays can be converted to Python scalars` from
numpy 2.0 onwards (it was a DeprecationWarning before that).

`set_ee_target`, `set_gains` and `enable`/`disable` all read the block back to
re-publish the fields they are not changing, so all three tripped over it and
every one of them failed inside the daemon with numpy 2.x. The existing
gain-persistence tests missed it because `snapshot_gains` / `restore_gains`
index correctly.

In-memory segment only -- no robot, no libfranka, no daemon.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.local_controller import LocalController  # noqa: E402
from frankatwin.shm_layout import SHM_TOTAL_BYTES, _build_view  # noqa: E402


class _StubController:
    """Just enough of LocalController: these methods only touch `_view`."""

    def __init__(self, view):
        self._view = view

    set_ee_target = LocalController.set_ee_target
    set_gains = LocalController.set_gains
    enable = LocalController.enable
    disable = LocalController.disable
    _set_enabled = LocalController._set_enabled


SEED = dict(
    kp_pos=200.0, kp_ori=20.0, kd_pos=1.5, kd_ori=0.5,
    error_delta_pos=0.05, error_delta_rot=0.30, enabled=True,
)
SEED_POS = [0.5, 0.0, 0.4]
SEED_QUAT = [1.0, 0.0, 0.0, 0.0]


@pytest.fixture
def ctl():
    view = _build_view(memoryview(bytearray(SHM_TOTAL_BYTES)))
    view.write_command(
        target_pos=np.asarray(SEED_POS, dtype=np.float64),
        target_quat=np.asarray(SEED_QUAT, dtype=np.float64),
        **SEED,
    )
    return _StubController(view)


def _read(ctl):
    c = ctl._view.read_command()
    out = {k: float(c[k][0]) for k in SEED if k != "enabled"}
    out["enabled"] = bool(c["enabled"][0])
    return out


def test_set_gains_changes_only_what_it_is_given(ctl):
    ctl.set_gains(kp_pos=500.0, error_delta_rot=0.80)

    got = _read(ctl)
    assert got["kp_pos"] == 500.0
    assert got["error_delta_rot"] == 0.80
    # everything else untouched
    for field in ("kp_ori", "kd_pos", "kd_ori", "error_delta_pos"):
        assert got[field] == SEED[field]
    assert got["enabled"] is True


def test_set_gains_keeps_the_target(ctl):
    ctl.set_gains(kp_pos=500.0)

    cmd = ctl._view.read_command()
    np.testing.assert_allclose(cmd["target_pos"][0], SEED_POS)
    np.testing.assert_allclose(cmd["target_quat"][0], SEED_QUAT)


def test_set_ee_target_keeps_the_gains(ctl):
    new_pos, new_quat = [0.3, 0.1, 0.5], [0.0, 1.0, 0.0, 0.0]
    ctl.set_ee_target(np.asarray(new_pos), np.asarray(new_quat))

    cmd = ctl._view.read_command()
    np.testing.assert_allclose(cmd["target_pos"][0], new_pos)
    np.testing.assert_allclose(cmd["target_quat"][0], new_quat)
    assert _read(ctl) == SEED


def test_enable_disable_keeps_everything_else(ctl):
    ctl.disable()
    got = _read(ctl)
    assert got["enabled"] is False
    assert {k: v for k, v in got.items() if k != "enabled"} == \
           {k: v for k, v in SEED.items() if k != "enabled"}

    ctl.enable()
    assert _read(ctl)["enabled"] is True

    cmd = ctl._view.read_command()
    np.testing.assert_allclose(cmd["target_pos"][0], SEED_POS)


def test_repeated_round_trips_do_not_drift(ctl):
    """Read-modify-write must be idempotent when nothing is passed."""
    for _ in range(5):
        ctl.set_gains()
    assert _read(ctl) == SEED
    cmd = ctl._view.read_command()
    np.testing.assert_allclose(cmd["target_pos"][0], SEED_POS)
    np.testing.assert_allclose(cmd["target_quat"][0], SEED_QUAT)
