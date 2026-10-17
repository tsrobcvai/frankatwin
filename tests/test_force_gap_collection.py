"""Exercise operator sequencing and recovery through the actual ring recorder."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from test_ring_log import _fresh_view, _StubController
from test_force_gap import complete_frame
from frankatwin.force_gap import real
from frankatwin.force_gap.protocol import Protocol
from frankatwin.force_gap.data import load_bundle
from frankatwin.local_controller import RobotState
from frankatwin.shm_layout import (
    _atomic_store_u64,
    OFFSET_HEADER,
    FRANKATWIN_SHM_STATE_FRAMES,
)


class RobotFixture(_StubController):
    def __init__(self, protocol, fault=None):
        super().__init__(_fresh_view())
        self.protocol, self.fault = protocol, fault
        self.calls = []
        self.elapsed = 0.0
        self.width = 0.08
        self.grasped = False
        self.recording = False
        self.frame = complete_frame()
        self.frame["q"] = protocol.q_init
        self.frame["ee_pos"] = [0.5, 0, 0.4]
        self.frame["ee_quat"] = [1, 0, 0, 0]

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.calls.append("close_client")

    def clock(self):
        return self.elapsed

    def sleep(self, seconds):
        self.elapsed += max(seconds, 0.00001)
        for _ in range(max(1, round(seconds * 1000))):
            seq = int(self._view.state_head) + 1
            cmd = self._view.read_command()
            self.frame["ee_pos"] = cmd["target_pos"][0]
            self.frame["ee_quat"] = cmd["target_quat"][0]
            self.frame["seq"] = seq
            self.frame["timestamp_s"] = seq / 1000
            self._view.states[seq % FRANKATWIN_SHM_STATE_FRAMES] = self.frame
            _atomic_store_u64(self._view.raw, OFFSET_HEADER + 24, seq)
        if self.recording:
            self._recorder.poll()

    def get_state(self, **kwargs):
        state = RobotState.from_frame(self.frame)
        if self.fault == "stale" and self.elapsed > 0.25:
            state.seq = 1
        if self.fault == "tracking" and self.elapsed > 0.25:
            state.ee_pos = state.ee_pos + [0.1, 0, 0]
        return state

    def move_to_q(self, target, **kwargs):
        self.calls.append("reset")
        np.testing.assert_equal(target, self.protocol.q_init)
        assert not self.grasped

    def set_gains(self, **kwargs):
        self.calls.append("gains")

    def enable(self):
        self.calls.append("enable")

    def gripper_open(self, **kwargs):
        self.calls.append("open")
        self.width, self.grasped = kwargs["width"], False

    def gripper_move(self, width, **kwargs):
        self.calls.append("empty_width")
        self.width = width

    def gripper_grasp(self, **kwargs):
        self.calls.append("grasp")
        self.width, self.grasped = kwargs["width"], True
        return {"result": True}

    def gripper_state(self):
        return {
            "width": self.width,
            "is_grasped": self.grasped
            and not (self.fault == "lost_grasp" and self.elapsed > 0.25),
        }

    def log_start(self, *args, **kwargs):
        self.calls.append("record")
        result = super().log_start(*args, **kwargs)
        self._recorder._stop_evt.set()
        self._recorder._thread.join()
        self._recorder._thread = None
        self.recording = True
        return result

    def log_stop(self, **kwargs):
        self.recording = False
        result = super().log_stop(**kwargs)
        if self.fault == "dropped_frames":
            result["dropped_frames"] = 1
        return result


def run_collection(tmp_path, monkeypatch, empty=False, fault=None):
    protocol = Protocol(
        rotation_waypoints_deg=[[0, 0, 0], [5, 0, 0], [0, 0, 0]],
        hold_seconds=0.2,
        move_seconds=0.2,
        settle_seconds=0.05,
    ).validate()
    path = tmp_path / "protocol.json"
    path.write_text(json.dumps(protocol.to_dict()))
    out = tmp_path / "collection"
    args = SimpleNamespace(
        protocol=path, out=out, dry_run=False, config=None, baseline=None, empty=empty
    )
    robot = RobotFixture(protocol, fault)
    prompts = []
    monkeypatch.setattr(real, "load_config", lambda _: None)

    def invoke():
        real.collect(
            args,
            client_factory=lambda _: robot,
            prompt=lambda message, token: prompts.append(token),
            clock=robot.clock,
            sleep=robot.sleep,
        )

    return invoke, robot, prompts, out


@pytest.mark.parametrize("empty", [False, True])
def test_guided_collection_saves_full_rate_wrench_and_operator_order(
    tmp_path, monkeypatch, empty
):
    invoke, robot, prompts, out = run_collection(tmp_path, monkeypatch, empty)
    invoke()
    assert prompts == ["RESET", "MODEL", "OPEN", "EMPTY" if empty else "GRASP", "RUN"]
    assert robot.calls == [
        "reset",
        "gains",
        "enable",
        "open",
        "empty_width" if empty else "grasp",
        "record",
        "close_client",
    ]
    a, m = load_bundle(out)
    assert m["completed"] and m["empty"] == empty
    assert len(a["t_s"]) >= 1000
    np.testing.assert_allclose(np.diff(a["t_s"]), 0.001, atol=1e-12)
    np.testing.assert_allclose(a["wrench_O_K"], a["O_F_ext_hat_K"])
    assert set(a["phase"]) == {0, 1, 2}
    assert (out / "ring_targets.csv").exists() and (out / "wrench_raw.csv").exists()
    assert len(json.loads((out / "events.json").read_text())) >= 50


@pytest.mark.parametrize("fault", ["tracking", "stale", "dropped_frames", "lost_grasp"])
def test_failed_collection_saves_diagnostics_and_keeps_gripper_closed(
    tmp_path, monkeypatch, fault
):
    invoke, robot, prompts, out = run_collection(tmp_path, monkeypatch, fault=fault)
    with pytest.raises(RuntimeError):
        invoke()
    a, m = load_bundle(out)
    assert not m["completed"] and len(a["t_s"]) > 3
    assert robot.calls.count("open") == 1 and robot.calls.index(
        "open"
    ) < robot.calls.index("grasp")
    assert not robot.recording
    if fault == "tracking":
        assert "tracking error" in m["exception"]
        np.testing.assert_allclose(
            robot._view.read_command()["target_pos"][0], robot.get_state().ee_pos
        )
    elif fault == "stale":
        assert "state stream stale" in m["exception"]
    elif fault == "lost_grasp":
        assert m["grasp_lost"]
    else:
        assert m["ring_log"]["dropped_frames"] == 1
