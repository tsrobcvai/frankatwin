"""Delayed hardware stops must not hold the daemon's arm command dispatcher."""

import json
import threading

import pytest

from test_gripper import _daemon_with_stub


def dispatch(daemon, op, **kwargs):
    return json.loads(daemon._dispatch(json.dumps(dict(op=op, **kwargs)).encode())[0])


def test_slow_stop_leaves_arm_commands_available_and_blocks_new_hand_motion():
    daemon = _daemon_with_stub(delay=5)
    daemon._gripper_stop_thread = None
    daemon._gripper_stop_last = {}
    entered, release = threading.Event(), threading.Event()

    def slow_stop():
        entered.set()
        assert release.wait(2), "test did not release the simulated hardware stop"
        daemon.controller.stop_evt.set()
        return {"result": True}

    daemon.controller.gripper_stop = slow_stop
    daemon.controller.gripper_latest_sample = lambda: None
    daemon.controller.set_ee_target = lambda pos, quat: 123
    motion = dispatch(daemon, "gripper_move", width=0.08)
    try:
        reply = dispatch(daemon, "gripper_stop_async")
        assert reply.get("ok") and reply.get("stopping")
        assert entered.wait(1)
        assert not release.is_set()
        # Even if the original motion has already ended, a stop still in flight
        # must not be allowed to hit a newly started move.
        daemon.controller.stop_evt.set()
        daemon._gripper_thread.join(1)
        assert not daemon._gripper_thread.is_alive()
        arm = dispatch(daemon, "set_ee_target", pos=[0.4, 0, 0.2], quat=[1, 0, 0, 0])
        assert arm == {"head": 123, "ok": True}
        state = dispatch(daemon, "gripper_latest_sample")
        assert state["busy"] and state["stopping"] and state["supports_async_stop"]
        # Duplicate cancellation is idempotent; it cannot start a second stop worker.
        thread = daemon._gripper_stop_thread
        assert dispatch(daemon, "gripper_stop_async")["stopping"]
        assert daemon._gripper_stop_thread is thread
        assert not dispatch(daemon, "gripper_move", width=0.04)["ok"]
    finally:
        release.set()
        daemon.controller.stop_evt.set()
        daemon._gripper_thread.join(2)
        if daemon._gripper_stop_thread is not None:
            daemon._gripper_stop_thread.join(2)
    state = dispatch(daemon, "gripper_latest_sample")
    assert not state["busy"] and not state["stopping"]
    assert state["last"]["seq"] == motion["seq"] and state["last"]["stopped"]
    assert state["stop_result"]["ok"]


@pytest.mark.parametrize("raises", [False, True])
def test_failed_async_stop_is_reported_and_prevents_new_motion(raises):
    daemon = _daemon_with_stub()
    daemon._gripper_stop_thread = None
    daemon._gripper_stop_last = {}
    daemon.controller.gripper_latest_sample = lambda: None

    def failure():
        if raises:
            raise RuntimeError("stop connection lost")
        return {"result": False}

    daemon.controller.gripper_stop = failure
    assert dispatch(daemon, "gripper_stop_async")["ok"]
    daemon._gripper_stop_thread.join(2)
    state = dispatch(daemon, "gripper_latest_sample")
    assert not state["stop_result"]["ok"]
    assert not dispatch(daemon, "gripper_move", width=0.08)["ok"]


def test_client_preserves_async_stop_capability_and_completion():
    from frankatwin.remote_client import FrankaTwinClient

    client = FrankaTwinClient.__new__(FrankaTwinClient)
    status = {"busy": True, "sample": None, "supports_async_stop": True,
              "stopping": True, "stop_result": {}, "last": {"seq": 2}}
    calls = []

    def call(request):
        calls.append(request)
        return status if request["op"] == "gripper_latest_sample" else {"stopping": True}

    client._call = call
    assert client.gripper_latest_sample() == status
    assert client.gripper_stop_async() == {"stopping": True}
    assert calls[-1] == {"op": "gripper_stop_async"}
