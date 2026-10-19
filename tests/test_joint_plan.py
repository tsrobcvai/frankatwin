"""Joint planning, command-mode switching, and bounded reference validation."""
import numpy as np
import pytest
from types import SimpleNamespace

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from frankatwin.force_gap.joint_plan import (
    JointPlan,
    flange_fk,
    default_ee_transform,
    Q_MIN,
    Q_MAX,
)
from frankatwin.force_gap.math import matrix_quaternion, quaternion_angle
from frankatwin.force_gap.protocol import Protocol
from frankatwin.local_controller import LocalController
from test_ring_log import _fresh_view


@pytest.fixture(scope="module")
def full_plan():
    return JointPlan.build(Protocol(), default_ee_transform())


def test_nominal_fk_matches_recorded_isaac_anchor():
    # Independent fixture: USD/PhysX actor poses in the original 2026-10-17 run.
    ee = flange_fk(Protocol().q_init) @ default_ee_transform()
    np.testing.assert_allclose(ee[:3, 3], [0.30689060688, 0, 0.47821086645], atol=1e-6)
    assert quaternion_angle(matrix_quaternion(ee[:3, :3]), [0, 1, 0, 0]) < 1e-6


def test_full_plan_tracks_ee_between_knots_and_returns_home(full_plan):
    p = Protocol()
    # Off-grid checks exercise interpolation, rather than only the IK samples.
    times = np.arange(0.013, p.duration, 0.137)
    q, dq = full_plan.sample(times)
    assert np.all(q > Q_MIN + 0.05) and np.all(q < Q_MAX - 0.05)
    assert np.max(np.abs(dq)) < p.max_joint_speed
    for t, angles in zip(times, q):
        pos, quat, _, _ = p.sample(t, full_plan.anchor_pos, full_plan.anchor_quat)
        ee = flange_fk(angles) @ full_plan.f_t_ee
        assert np.linalg.norm(ee[:3, 3] - pos) < 0.0002
        assert quaternion_angle(matrix_quaternion(ee[:3, :3]), quat) < 0.001
    np.testing.assert_allclose(full_plan.q[-1], p.q_init, atol=1e-5)
    np.testing.assert_allclose(full_plan.sample([0, p.duration])[1], 0, atol=1e-12)
    assert np.rad2deg(np.max(np.abs(q[:, 0] - p.q_init[0]))) < 10
    eps = 1e-5
    numerical = (
        full_plan.sample(times + eps)[0] - full_plan.sample(times - eps)[0]
    ) / (2 * eps)
    np.testing.assert_allclose(numerical, dq, atol=1e-7)


def test_planner_rejects_incompatible_robot_frame():
    with pytest.raises(ValueError, match="FK disagrees"):
        JointPlan.build(Protocol(), np.eye(4), [0, 0, 0], [1, 0, 0, 0])


def test_planner_rejects_too_fast_motion():
    p = Protocol(
        rotation_waypoints_deg=[[0, 0, 0], [30, 0, 0], [0, 0, 0]], move_seconds=0.1
    )
    with pytest.raises(ValueError, match="speed/acceleration"):
        JointPlan.build(p, default_ee_transform())


def test_joint_command_is_atomic_and_enable_preserves_mode():
    ctl = object.__new__(LocalController)
    ctl._shm = SimpleNamespace(view=_fresh_view())
    p = Protocol()
    ctl.set_joint_target(
        p.q_init, np.zeros(7), p.joint_kp, p.joint_kd, [0.3, 0, 0.4], [0, 1, 0, 0]
    )
    ctl.disable()
    ctl.set_gains(kp_pos=300)
    ctl.enable()
    cmd = ctl._view.read_command()[0]
    assert cmd["control_mode"] == 1 and cmd["enabled"] == 1
    np.testing.assert_equal(cmd["target_q"], p.q_init)
    np.testing.assert_equal(cmd["joint_kp"], p.joint_kp)
    with pytest.raises(ValueError, match="finite"):
        ctl.set_joint_target(
            [np.nan] * 7,
            np.zeros(7),
            p.joint_kp,
            p.joint_kd,
            [0.3, 0, 0.4],
            [0, 1, 0, 0],
        )
    assert ctl._view.read_command()["seq"][0] == cmd["seq"]
    ctl.set_ee_target(np.array([0.3, 0, 0.4]), np.array([0, 1, 0, 0]))
    assert ctl._view.read_command()["control_mode"][0] == 0
