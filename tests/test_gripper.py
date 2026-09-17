"""Gripper path without hardware: config keys, gripper_cmd arguments, output
parsing, and the daemon's asynchronous gripper ops against a stub controller."""

from __future__ import annotations

import pathlib
import sys
import threading
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.config import DEFAULT_CONFIG_PATH, GripperConfig, load_config  # noqa: E402
from frankatwin.local_controller import gripper_command_args, parse_gripper_output  # noqa: E402


# ----------------------------------------------------------------------------- config
def test_default_gripper_config_matches_yaml():
    cfg = load_config(DEFAULT_CONFIG_PATH)
    g = cfg.gripper
    assert g.enabled is True
    # 0.1 m/s is the Franka Hand's width-closing ceiling (50 mm/s per finger,
    # both fingers move); grasp_speed used to be 0.5, which was over the spec.
    assert (g.move_speed, g.grasp_speed, g.grasp_force) == (0.1, 0.1, 70.0)
    assert (g.grasp_width, g.epsilon_inner, g.epsilon_outer, g.max_width) == (-0.01, 0.08, 0.08, 0.08)


@pytest.mark.parametrize("key,value,msg", [
    ("grasp_force", 80.0, "grasp_force"),       # above the Franka Hand's 70 N
    ("grasp_force", 0.0, "grasp_force"),
    ("grasp_width", 0.09, "grasp_width"),       # beyond the stroke
    ("epsilon_inner", -0.01, "epsilon_inner"),
    ("move_speed", 0.0, "move_speed"),
])
def test_gripper_config_validation(tmp_path, key, value, msg):
    text = DEFAULT_CONFIG_PATH.read_text()
    import re
    text, n = re.subn(rf"^  {key}:.*$", f"  {key}: {value}", text, flags=re.M)
    assert n == 1
    path = tmp_path / "robot.yaml"
    path.write_text(text)
    with pytest.raises(ValueError, match=msg):
        load_config(path)


# ----------------------------------------------------------------------------- args
def test_gripper_args_defaults_from_config():
    g = GripperConfig()
    assert gripper_command_args(g, "homing") == ["homing"]
    assert gripper_command_args(g, "stop") == ["stop"]
    assert gripper_command_args(g, "state") == ["state"]
    assert gripper_command_args(g, "move") == ["move", "--width", "0.08000", "--speed", "0.1000"]
    assert gripper_command_args(g, "grasp") == [
        "grasp", "--width", "-0.01000", "--speed", "0.1000", "--force", "70.00",
        "--eps-in", "0.0800", "--eps-out", "0.0800",
    ]


def test_gripper_args_overrides_and_limits():
    g = GripperConfig()
    assert gripper_command_args(g, "move", width=0.0336, speed=0.05)[2:] == ["0.03360", "--speed", "0.0500"]
    a = gripper_command_args(g, "grasp", width=0.008, force=30, epsilon_inner=0.005, epsilon_outer=0.005)
    assert a[2] == "0.00800" and a[6] == "30.00" and a[8] == a[10] == "0.0050"
    with pytest.raises(ValueError, match="width"):
        gripper_command_args(g, "move", width=0.09)
    with pytest.raises(ValueError, match="force"):
        gripper_command_args(g, "grasp", force=71)
    with pytest.raises(ValueError, match="unknown"):
        gripper_command_args(g, "wiggle")


# ----------------------------------------------------------------------------- output
def test_parse_gripper_output_takes_last_json_line():
    out = ('[gripper_cmd] connecting\n'
           '{"ok":true,"cmd":"grasp","result":true,"stopped":false,'
           '"state":{"width":0.0102,"max_width":0.0800,"is_grasped":true,"temperature":32}}\n')
    d = parse_gripper_output(out)
    assert d["ok"] and d["cmd"] == "grasp" and d["state"]["width"] == pytest.approx(0.0102)
    with pytest.raises(RuntimeError, match="nothing"):
        parse_gripper_output("")
    with pytest.raises(RuntimeError, match="not JSON"):
        parse_gripper_output("Usage: ...")


# ----------------------------------------------------------------------------- daemon ops
class _StubController:
    """Stands in for LocalController: gripper calls block for `delay` seconds."""

    def __init__(self, delay: float = 0.2):
        self.delay = delay
        self.calls = []
        self.stop_evt = threading.Event()

    def _blocking(self, cmd, **kw):
        self.calls.append((cmd, kw))
        self.stop_evt.wait(self.delay)
        stopped = self.stop_evt.is_set()
        return {"ok": True, "cmd": cmd, "result": not stopped, "stopped": stopped,
                "state": {"width": 0.01, "max_width": 0.08, "is_grasped": not stopped, "temperature": 30}}

    def gripper_homing(self):
        return self._blocking("homing")

    def gripper_move(self, width, speed=None):
        return self._blocking("move", width=width, speed=speed)

    def gripper_grasp(self, **kw):
        return self._blocking("grasp", **kw)

    def gripper_stop(self):
        self.stop_evt.set()
        return {"ok": True, "cmd": "stop", "result": True, "stopped": True}

    def gripper_state(self):
        return {"width": 0.05, "max_width": 0.08, "is_grasped": False, "temperature": 30}


def _daemon_with_stub(delay=0.2):
    """A FrankaTwinDaemon instance with the ZMQ/controller __init__ bypassed."""
    from frankatwin.daemon import FrankaTwinDaemon
    d = FrankaTwinDaemon.__new__(FrankaTwinDaemon)
    d.controller = _StubController(delay)
    d._op_lock = threading.Lock()
    d._gripper_thread = None
    d._gripper_lock = threading.Lock()
    d._gripper_seq = 0
    d._gripper_last = {}
    return d


def test_daemon_gripper_ops_are_async_and_pollable():
    d = _daemon_with_stub(delay=0.3)
    t0 = time.monotonic()
    r = d._op_gripper_grasp({"force": 30.0})
    assert r == {"started": True, "seq": 1} and time.monotonic() - t0 < 0.1   # returned before the 0.3 s call ended
    assert d._op_gripper_state({})["busy"] is True
    # a second command while busy is refused instead of queued
    with pytest.raises(RuntimeError, match="busy"):
        d._op_gripper_move({"width": 0.08})
    d._gripper_thread.join(2.0)
    st = d._op_gripper_state({})
    assert st["busy"] is False and st["last"]["seq"] == 1 and st["last"]["result"] is True
    assert d.controller.calls[0] == ("grasp", {"width": None, "speed": None, "force": 30.0,
                                               "epsilon_inner": None, "epsilon_outer": None})
    # fresh read once idle
    assert d._op_gripper_state({"fresh": True})["state"]["width"] == pytest.approx(0.05)


def test_daemon_gripper_stop_interrupts_running_command():
    d = _daemon_with_stub(delay=5.0)
    r = d._op_gripper_move({"width": 0.08, "speed": 0.05})
    assert d._op_gripper_state({})["busy"] is True
    t0 = time.monotonic()
    assert d._op_gripper_stop({}) == {"result": True}
    assert time.monotonic() - t0 < 2.0
    last = d._op_gripper_state({})["last"]
    assert last["seq"] == r["seq"] and last["stopped"] is True and last["result"] is False


def test_daemon_gripper_failure_is_reported_via_state():
    d = _daemon_with_stub()

    def boom():
        raise RuntimeError("gripper_cmd grasp failed (exit 10): Connection to FCI refused")
    d.controller.gripper_homing = boom
    d._op_gripper_homing({})
    d._gripper_thread.join(2.0)
    last = d._op_gripper_state({})["last"]
    assert last["ok"] is False and "refused" in last["error"] and last["cmd"] == "homing"
