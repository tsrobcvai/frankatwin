"""Independent physical fixtures and telemetry/analysis contracts; no robot required."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from frankatwin.force_gap.math import (
    rotation,
    rotvec_quat,
    multiply,
    matrix_quaternion,
    transform_wrench,
    support_wrench,
    box_inertia,
    quaternion_angle,
    validate_transform,
    interpolate_quaternions,
)
from frankatwin.force_gap.protocol import Protocol
from frankatwin.force_gap.data import save_bundle, derive_frames, load_bundle
from frankatwin.force_gap.real import require_wrench_state, normalize_log
from frankatwin.local_controller import RobotState
from frankatwin.daemon import _state_to_dict
from frankatwin.remote_client import _state_from_dict
from frankatwin.shm_layout import STATE_FRAME_DTYPE, WRENCH_STATE_FIELDS
from frankatwin.ring_log import MERGED_COLUMNS, WRENCH_COLUMNS, RunRecorder


def test_wrench_rotation_and_point_shift():
    rz = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    actual = transform_wrench([1, 0, 0, 0, 0, 0], rz, [0, 0, 1])
    np.testing.assert_allclose(actual, [0, 1, 0, -1, 0, 0], atol=1e-12)
    inverse = transform_wrench(actual, rz.T, [0, 0, -1])
    np.testing.assert_allclose(inverse, [1, 0, 0, 0, 0, 0], atol=1e-12)


def test_gravity_support_sign_and_moment():
    # 310 g, 50 mm +X offset: upward support causes -Y torque.
    w = support_wrench(0.31, np.array([0.05, 0, 0]), np.zeros(3))
    np.testing.assert_allclose(w, [0, 0, 3.0411, 0, -0.152055, 0], atol=1e-10)
    # In free fall no support is needed.
    np.testing.assert_allclose(
        support_wrench(0.31, np.zeros(3), np.zeros(3), [0, 0, -9.81]), 0
    )


def test_inertial_torque_and_parallel_axis_shift():
    inertia = box_inertia(0.31, [0.05, 0.05, 0.1])
    w = support_wrench(
        0.31,
        np.array([0.1, 0, 0]),
        np.zeros(3),
        [0, 2, 0],
        np.eye(3),
        inertia,
        [0, 0, 0],
        [0, 0, 3],
        gravity=[0, 0, 0],
    )
    np.testing.assert_allclose(
        w, [0, 0.62, 0, 0, 0, 0.062 + 3 * 0.31 * 0.005 / 12], atol=1e-12
    )


@pytest.mark.parametrize(
    "axis",
    [
        np.array([1.0, 0, 0]),
        np.array([0.0, 1, 0]),
        np.array([0.0, 0, 1]),
        np.ones(3) / np.sqrt(3),
    ],
)
@pytest.mark.parametrize("angle", [0.0, 0.2, np.pi - 0.00001, np.pi])
def test_matrix_quaternion_near_pi(axis, angle):
    q = rotvec_quat(axis * angle)
    r = rotation(q)
    np.testing.assert_allclose(rotation(matrix_quaternion(r)), r, atol=1e-10)


def test_trajectory_holds_endpoints_and_continuity():
    p = Protocol().validate()
    pos = [0.4, 0, 0.4]
    quat = [0, 1, 0, 0]
    assert p.duration == 50
    np.testing.assert_allclose(p.sample(0, pos, quat)[1], quat)
    np.testing.assert_allclose(p.sample(p.duration, pos, quat)[1], quat)
    assert not p.sample(0.1, pos, quat)[3]
    assert p.sample(1.0, pos, quat)[3]
    boundary = p.hold_seconds + p.move_seconds
    a = p.sample(boundary - 1e-5, pos, quat)[1]
    b = p.sample(boundary + 1e-5, pos, quat)[1]
    assert quaternion_angle(a, b) < 1e-7
    assert p.sample(boundary + 1, pos, quat)[2:] == (1, True)
    p.rotation_waypoints_deg[-1] = [1, 0, 0]
    with pytest.raises(ValueError):
        p.validate()


def test_transform_rejects_row_major_confusion_and_reflection():
    t = np.eye(4)
    t[3, 0] = 0.1
    with pytest.raises(ValueError):
        validate_transform(t)
    t = np.eye(4)
    t[0, 0] = -1
    with pytest.raises(ValueError):
        validate_transform(t)


def test_quaternion_interpolation_handles_sign_flips_and_rotation():
    q0 = np.array([1.0, 0, 0, 0])
    q1 = rotvec_quat([0, 0, np.pi / 2])
    q = interpolate_quaternions([0, 1, 2], [q0, -q0, q1], [0.5, 1.5])
    np.testing.assert_allclose(rotation(q[0]), np.eye(3), atol=1e-12)
    np.testing.assert_allclose(
        rotation(q[1]), rotation(rotvec_quat([0, 0, np.pi / 4])), atol=1e-12
    )


def complete_frame():
    frame = np.zeros((), dtype=STATE_FRAME_DTYPE)
    frame["ee_quat"] = [1, 0, 0, 0]
    for name, width in WRENCH_STATE_FIELDS.items():
        frame[name] = np.arange(width) + 0.125 if width > 1 else 0.25
    frame["EE_T_K"] = np.eye(4).flatten(order="F")
    f = np.eye(4)
    f[:3, 3] = [0.01, 0.02, 0.1034]
    frame["F_T_EE"] = f.flatten(order="F")
    return frame


def test_wrench_state_cpp_numpy_wire_roundtrip():
    frame = complete_frame()
    state = RobotState.from_frame(frame)
    require_wrench_state(state)
    decoded = _state_from_dict(json.loads(json.dumps(_state_to_dict(state))))
    for name in WRENCH_STATE_FIELDS:
        np.testing.assert_equal(getattr(decoded, name), frame[name])
    old = _state_to_dict(state)
    for name in WRENCH_STATE_FIELDS:
        old.pop(name)
    with pytest.raises(RuntimeError, match="shm v5"):
        require_wrench_state(_state_from_dict(old))


def real_tables(n=20):
    merged = np.zeros((n, len(MERGED_COLUMNS)))
    wrench = np.zeros((n, len(WRENCH_COLUMNS)))
    mc = {k: i for i, k in enumerate(MERGED_COLUMNS)}
    wc = {k: i for i, k in enumerate(WRENCH_COLUMNS)}
    merged[:, mc["seq"]] = wrench[:, wc["seq"]] = np.arange(100, 100 + n)
    wrench[:, wc["timestamp_s"]] = 20 + np.arange(n) * 0.001
    merged[:, mc["quat_w"]] = merged[:, mc["quat_des_w"]] = 1
    frame = complete_frame()
    for name, width in WRENCH_STATE_FIELDS.items():
        if width == 1:
            wrench[:, wc[name]] = frame[name]
        else:
            for j in range(width):
                wrench[:, wc[f"{name}_{j}"]] = frame[name][j]
    return merged, wrench


def test_normalize_joins_sequences_and_preserves_raw_estimates():
    merged, wrench = real_tables()
    meta = {"empty": False}
    a = normalize_log(merged, wrench, [{"phase": 0, "settled": True}], Protocol(), meta)
    np.testing.assert_allclose(a["t_s"], np.arange(20) * 0.001, atol=1e-12)
    np.testing.assert_allclose(a["wrench_O_K"], a["O_F_ext_hat_K"])
    np.testing.assert_allclose(a["wrench_K_K"], a["O_F_ext_hat_K"])
    wrench[2, 0] += 1
    with pytest.raises(ValueError, match="sequence"):
        normalize_log(merged, wrench, [], Protocol(), meta)


def test_normalize_rejects_midrun_payload_change():
    merged, wrench = real_tables()
    wrench[-1, WRENCH_COLUMNS.index("m_load")] += 0.31
    with pytest.raises(ValueError, match="m_load"):
        normalize_log(merged, wrench, [], Protocol(), {"empty": False})


def test_every_v5_field_has_cpp_pinned_offset():
    # Detailed C++ comparison is in test_shm_layout; check the extension is contiguous.
    offset = 344
    for name, width in WRENCH_STATE_FIELDS.items():
        assert STATE_FRAME_DTYPE.fields[name][1] == offset
        offset += 8 * width
    assert offset == 1104


def synthetic_bundle(path, source, bias=0.0):
    p = Protocol()
    n = 100
    t = np.linspace(0, 1, n)
    a = {
        "t_s": t,
        "phase": np.zeros(n, int),
        "settled": np.ones(n, bool),
        "q": np.zeros((n, 7)),
        "ee_pos": np.tile([0.4, 0, 0.4], (n, 1)),
        "ee_quat": np.tile([1, 0, 0, 0], (n, 1)),
        "wrench_O_K": np.tile([0, 0, 3.0411, 0, 0, 0], (n, 1)),
    }
    a["wrench_O_K"][:, 0] += bias
    derive_frames(a, p, np.eye(4))
    m = {
        "source": source,
        "protocol": p.to_dict(),
        "protocol_sha256": p.digest,
        "completed": True,
        "empty": False,
        "EE_T_K": np.eye(4).tolist(),
        "F_T_EE": np.eye(4).tolist(),
    }
    save_bundle(path, a, m)


def test_comparison_recovers_known_bias_without_fitting(tmp_path):
    pytest.importorskip("matplotlib")
    from frankatwin.force_gap.report import compare

    synthetic_bundle(tmp_path / "s", "sim", 0.2)
    synthetic_bundle(tmp_path / "r", "real")
    result = compare(tmp_path / "s", tmp_path / "r", tmp_path / "comparison")
    np.testing.assert_allclose(
        result["time_aligned"]["wrench_O_K"]["bias"], [0.2, 0, 0, 0, 0, 0], atol=1e-12
    )
    assert result["automatic_time_alignment"] is False
    assert (tmp_path / "comparison/gap_K_K.png").stat().st_size > 1000


def test_analysis_distinguishes_finished_trajectory_from_settled_poses(tmp_path):
    pytest.importorskip("matplotlib")
    from frankatwin.force_gap.report import analyze

    synthetic_bundle(tmp_path / "s", "sim")
    report = analyze(tmp_path / "s")
    assert report["completed"] is True
    assert report["all_poses_settled"] is False
    assert report["unsettled_pose_indices"] == list(range(1, 9))


def test_comparison_rejects_wrong_frame_and_incomplete(tmp_path):
    from frankatwin.force_gap.report import compare

    synthetic_bundle(tmp_path / "s", "sim")
    synthetic_bundle(tmp_path / "r", "real")
    path = tmp_path / "r/metadata.json"
    meta = json.loads(path.read_text())
    meta["EE_T_K"][0][3] = 0.01
    path.write_text(json.dumps(meta))
    with pytest.raises(ValueError, match="EE_T_K"):
        compare(tmp_path / "s", tmp_path / "r", tmp_path / "a")
    meta["EE_T_K"][0][3] = 0
    meta["completed"] = False
    path.write_text(json.dumps(meta))
    with pytest.raises(ValueError, match="incomplete"):
        compare(tmp_path / "s", tmp_path / "r", tmp_path / "b")
