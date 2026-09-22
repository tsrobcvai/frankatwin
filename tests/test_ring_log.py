"""1 kHz ring log: lossless drain, tick-exact targets, drop-in CSV schema.

Drives an in-memory copy of the shm segment with a Python stand-in for the
C++ producer (`_publish` mirrors `state_publish` in src/shm_layout.h: fill
slot (head + 1) % N, then store the new head). No robot. The last test runs the
daemon's REP loop and a FrankaTwinClient over loopback ZMQ to move a log from
one to the other; the controller behind the daemon is a stub.
"""

from __future__ import annotations

import pathlib
import sys
import threading
import types

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.local_controller import LocalController  # noqa: E402
from frankatwin.ring_log import (  # noqa: E402
    CART_IMPEDANCE_COLUMNS,
    MERGED_COLUMNS,
    TARGETS_COLUMNS,
    RingDrainer,
    RunRecorder,
    TargetEvent,
    merge_run,
    targets_path_for,
)
from frankatwin.shm_layout import (  # noqa: E402
    FRANKATWIN_SHM_STATE_FRAMES as N_RING,
    OFFSET_HEADER,
    SHM_TOTAL_BYTES,
    _atomic_store_u64,
    _build_view,
)

# Header of examples/cart_impedance.py `_write_csv`, copied verbatim so a
# schema drift on either side fails here.
CART_IMPEDANCE_HEADER = (
    "t_s,period_ms,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,"
    "x_x,x_y,x_z,quat_x,quat_y,quat_z,quat_w,"
    "x_des_x,x_des_y,x_des_z,dx_des_x,dx_des_y,dx_des_z,"
    "quat_des_x,quat_des_y,quat_des_z,quat_des_w,"
    "tau1,tau2,tau3,tau4,tau5,tau6,tau7,tau_J1,tau_J2,tau_J3,tau_J4,tau_J5,tau_J6,tau_J7"
)


# --------------------------------------------------------------------------- producer
class _Producer:
    """Stand-in for osc_shm's 1 kHz loop. Frame `seq` encodes itself in q[0]
    and timestamp_s so the drained data can be checked against the seq."""

    def __init__(self, view):
        self.view = view

    @property
    def head(self) -> int:
        return int(self.view.state_head)

    def publish(self, k: int = 1) -> None:
        for _ in range(k):
            nxt = self.head + 1
            slot = self.view.states[nxt % N_RING]
            slot["seq"] = nxt
            slot["timestamp_s"] = nxt * 1.0e-3
            slot["q"] = np.full(7, float(nxt))
            slot["dq"] = np.full(7, -float(nxt))
            slot["ee_pos"] = [0.5, 0.0, 0.4 + 1e-6 * nxt]
            slot["ee_quat"] = [1.0, 0.0, 0.0, 0.0]          # wxyz
            slot["tau"] = np.full(7, 0.1)
            slot["tau_J"] = np.full(7, 0.2)
            _atomic_store_u64(self.view.raw, OFFSET_HEADER + 24, nxt)


def _fresh_view():
    view = _build_view(memoryview(bytearray(SHM_TOTAL_BYTES)))
    view.write_command(
        target_pos=np.array([0.5, 0.0, 0.4]), target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        kp_pos=200.0, kp_ori=20.0, kd_pos=0.0, kd_ori=0.0, error_delta_pos=0.0, enabled=True,
    )
    return view


class _StubController:
    """Just enough of LocalController for the setpoint path."""

    def __init__(self, view):
        self._view = view
        self._recorder = None

    set_ee_target = LocalController.set_ee_target
    log_start = LocalController.log_start
    log_stop = LocalController.log_stop
    log_fetch = LocalController.log_fetch
    log_save = LocalController.log_save
    log_id = LocalController.log_id
    log_active = LocalController.log_active
    _finished_log = LocalController._finished_log

    # What else FrankaTwinDaemon.run() touches.
    def get_state(self):
        return None

    def ensure_running(self):
        return False

    def close(self):
        pass


# --------------------------------------------------------------------------- drainer
def test_drainer_is_lossless_across_wraparound():
    view = _fresh_view()
    prod = _Producer(view)
    dr = RingDrainer(view)
    rng = np.random.default_rng(0)
    total = 0
    while total < 3 * N_RING:                       # three laps of the ring
        k = int(rng.integers(1, 40))                # 1-39 frames between polls (~10 at 100 Hz)
        prod.publish(k)
        total += k
        dr.poll()
    frames = dr.frames()
    assert dr.gaps == [] and dr.dropped == 0 and dr.resets == 0
    np.testing.assert_array_equal(frames["seq"], np.arange(1, total + 1))
    np.testing.assert_array_equal(frames["q"][:, 0], np.arange(1, total + 1))


def test_drainer_reports_a_gap_when_lapped():
    view = _fresh_view()
    prod = _Producer(view)
    dr = RingDrainer(view)
    prod.publish(10)
    dr.poll()
    prod.publish(N_RING + 500)                      # more than one ring between polls
    dr.poll()
    frames = dr.frames()
    assert dr.gaps == [(11, 10 + 500)]
    assert dr.dropped == 500
    # what survives is exactly the newest ring's worth
    np.testing.assert_array_equal(frames["seq"], np.r_[np.arange(1, 11), np.arange(511, 10 + N_RING + 501)])


def test_drainer_survives_a_producer_reset():
    view = _fresh_view()
    prod = _Producer(view)
    dr = RingDrainer(view)
    prod.publish(50)
    dr.poll()
    _atomic_store_u64(view.raw, OFFSET_HEADER + 24, 0)   # osc_shm --init-shm
    prod.publish(5)
    dr.poll()
    assert dr.resets == 1
    assert dr.frames()["seq"][-5:].tolist() == [1, 2, 3, 4, 5]


# --------------------------------------------------------------------------- merge
def test_merge_targets_are_tick_exact_and_start_snapshot_fills_the_prefix():
    view = _fresh_view()
    prod = _Producer(view)
    dr = RingDrainer(view)
    prod.publish(20)                                # frames 1..20 under the start target
    events = []
    for i, head in enumerate((20, 45, 45, 70)):    # two writes inside one tick at 45
        while prod.head < head:
            prod.publish()
        events.append(TargetEvent(idx=i, head=head, pos=np.array([i + 1.0, 0.0, 0.0]),
                                  quat_wxyz=np.array([0.0, 0.0, 0.0, 1.0])))  # wxyz -> z-quat
    prod.publish(30)
    dr.poll()
    table = merge_run(dr.frames(), events, np.array([9.0, 9.0, 9.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    col = {c: i for i, c in enumerate(MERGED_COLUMNS)}
    seq = table[:, col["seq"]].astype(int)
    xdes = table[:, col["x_des_x"]]
    tidx = table[:, col["target_idx"]].astype(int)

    assert seq.tolist() == list(range(1, 101))
    # start snapshot until the first target takes effect at head + 1 = 21
    assert np.all(xdes[seq <= 20] == 9.0) and np.all(tidx[seq <= 20] == -1)
    assert np.all(xdes[(seq >= 21) & (seq <= 45)] == 1.0) and np.all(tidx[(seq >= 21) & (seq <= 45)] == 0)
    # two writes at head 45: the later one (idx 2) is what the controller saw from 46
    assert np.all(xdes[(seq >= 46) & (seq <= 70)] == 3.0) and np.all(tidx[(seq >= 46) & (seq <= 70)] == 2)
    assert np.all(xdes[seq >= 71] == 4.0)
    # t_s is the tick count from the first frame; period_ms from the producer clock
    np.testing.assert_allclose(table[:, col["t_s"]], (seq - 1) * 1e-3)
    np.testing.assert_allclose(table[1:, col["period_ms"]], 1.0)
    assert table[0, col["period_ms"]] == 0.0
    # quaternions come out xyzw: start (1,0,0,0) wxyz -> (0,0,0,1); target wxyz (0,0,0,1) -> (0,0,1,0)
    np.testing.assert_allclose(table[0, [col["quat_des_x"], col["quat_des_y"], col["quat_des_z"], col["quat_des_w"]]], [0, 0, 0, 1])
    np.testing.assert_allclose(table[-1, [col["quat_des_x"], col["quat_des_y"], col["quat_des_z"], col["quat_des_w"]]], [0, 0, 1, 0])
    np.testing.assert_allclose(table[:, [col["quat_x"], col["quat_y"], col["quat_z"], col["quat_w"]]][0], [0, 0, 0, 1])
    # no velocity feedforward in the command block
    assert np.all(table[:, [col["dx_des_x"], col["dx_des_y"], col["dx_des_z"]]] == 0.0)
    # state columns are the frames'
    np.testing.assert_allclose(table[:, col["q1"]], seq)
    np.testing.assert_allclose(table[:, col["dq7"]], -seq)
    np.testing.assert_allclose(table[:, col["tau_J3"]], 0.2)


def test_merge_of_nothing_is_an_empty_table():
    t = merge_run(np.empty(0, dtype=_fresh_view().states.dtype), [], np.zeros(3), np.array([1.0, 0, 0, 0]))
    assert t.shape == (0, len(MERGED_COLUMNS))


# --------------------------------------------------------------------------- schema
def test_merged_columns_are_the_cart_impedance_log_plus_seq_and_target_idx():
    assert ",".join(CART_IMPEDANCE_COLUMNS) == CART_IMPEDANCE_HEADER
    assert MERGED_COLUMNS == CART_IMPEDANCE_COLUMNS + ["seq", "target_idx"]


# --------------------------------------------------------------------------- session
def _compute_step_counts(t_csv, sim_dt=1e-3):
    """Copy of sysid_franka_osc.compute_step_counts (the consumer's time base)."""
    n = len(t_csv)
    steps = np.ones(n, dtype=np.int64)
    if n >= 2:
        steps[: n - 1] = np.clip(np.round(np.diff(t_csv) / sim_dt).astype(np.int64), 1, None)
    return steps


def test_session_through_set_ee_target_writes_a_sim_readable_csv(tmp_path):
    view = _fresh_view()
    prod = _Producer(view)
    ctl = _StubController(view)
    prod.publish(300)                               # frames before the session are skipped

    info = ctl.log_start(tmp_path / "run_1khz.csv")
    rec = ctl._recorder
    rec._stop_evt.set(); rec._thread.join()        # drive the drainer by hand below
    rec._thread = None
    assert info["seq_start"] == 300 and ctl.log_active

    # 50 Hz setpoints for 0.4 s: 20 frames per target, drained every ~7 frames.
    heads = []
    for k in range(20):
        heads.append(ctl.set_ee_target(np.array([0.5, 0.01 * k, 0.4]), np.array([1.0, 0.0, 0.0, 0.0])))
        for _ in range(3):
            prod.publish(7 if _ < 2 else 6)
            rec.poll()
    assert heads == [300 + 20 * k for k in range(20)]

    summary = ctl.log_stop()
    assert not ctl.log_active
    assert summary["num_frames"] == 400 and summary["num_targets"] == 20
    assert summary["seq_first"] == 301 and summary["seq_last"] == 700
    assert summary["gaps"] == [] and summary["dropped_frames"] == 0 and summary["resets"] == 0
    assert summary["duration_s"] == pytest.approx(0.4)

    # Read it back the way sysid_franka_osc.load_real_data does.
    arr = np.genfromtxt(summary["path"], delimiter=",", names=True, dtype=np.float64)
    assert list(arr.dtype.names) == MERGED_COLUMNS
    assert arr.shape[0] == 400
    assert _compute_step_counts(arr["t_s"]).tolist() == [1] * 400     # one tick per row
    x_des_y = arr["x_des_y"]
    seq = arr["seq"].astype(int)
    for k in range(20):
        eff = 300 + 20 * k + 1
        rows = (seq >= eff) & (seq < eff + 20)
        np.testing.assert_allclose(x_des_y[rows], 0.01 * k)         # target k for exactly its 20 ticks
        assert np.all(arr["target_idx"][rows] == k)
    assert arr["t_s"][0] == 0.0 and seq[0] == 301
    assert np.all(np.isfinite(arr["tau_J1"]))

    tp = targets_path_for(summary["path"])
    assert tp == tmp_path / "run_1khz_targets.csv" and summary["targets_path"] == str(tp)
    tv = np.genfromtxt(tp, delimiter=",", names=True, dtype=np.float64)
    assert list(tv.dtype.names) == TARGETS_COLUMNS
    assert tv["head"].astype(int).tolist() == heads
    assert tv["seq_effective"].astype(int).tolist() == [h + 1 for h in heads]


def test_gap_in_the_log_becomes_a_longer_hold_for_the_sim(tmp_path):
    view = _fresh_view()
    prod = _Producer(view)
    rec = RunRecorder(view, tmp_path / "gap.csv")
    prod.publish(10); rec.poll()
    prod.publish(N_RING + 5); rec.poll()            # lapped once: 5 frames lost
    summary = rec.stop()
    assert summary["dropped_frames"] == 5 and summary["gaps"] == [[11, 15]]
    arr = np.genfromtxt(summary["path"], delimiter=",", names=True, dtype=np.float64)
    steps = _compute_step_counts(arr["t_s"])
    assert steps[9] == 6 and np.all(np.delete(steps, 9) == 1)


def test_second_session_is_refused_while_one_runs(tmp_path):
    ctl = _StubController(_fresh_view())
    ctl.log_start(tmp_path / "a.csv")
    with pytest.raises(RuntimeError):
        ctl.log_start(tmp_path / "b.csv")
    ctl.log_stop()
    with pytest.raises(RuntimeError):
        ctl.log_stop()


def test_session_without_a_path_stays_in_memory_until_fetched_or_saved(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    view = _fresh_view()
    prod = _Producer(view)
    ctl = _StubController(view)
    with pytest.raises(RuntimeError):                                # nothing recorded yet
        ctl.log_fetch()

    info = ctl.log_start()                                           # what the daemon does
    assert info["path"] is None and info["log_id"] == 1
    ctl.set_ee_target(np.array([0.5, 0.1, 0.4]), np.array([1.0, 0.0, 0.0, 0.0]))
    prod.publish(30)
    summary = ctl.log_stop(save=False)
    assert summary["path"] is None and summary["targets_path"] is None
    assert summary["num_frames"] == 30 and summary["log_id"] == 1
    assert list(tmp_path.iterdir()) == []                            # nothing on this machine

    merged = ctl.log_fetch("merged")
    assert merged.shape == (30, len(MERGED_COLUMNS))
    np.testing.assert_array_equal(ctl.log_fetch("merged", 10, 5), merged[10:15])
    assert ctl.log_fetch("merged", 25, 100).shape[0] == 5            # clipped at the end
    assert ctl.log_fetch("merged", 30, 10).shape[0] == 0
    assert ctl.log_fetch("targets").shape == (1, len(TARGETS_COLUMNS))
    with pytest.raises(ValueError):
        ctl.log_fetch("frames")
    with pytest.raises(ValueError):
        ctl.log_fetch("merged", -1)
    with pytest.raises(ValueError):                                  # no path, here or at log_start
        ctl.log_save()

    written = ctl.log_save(tmp_path / "sub" / "later.csv")
    assert written == {"path": str(tmp_path / "sub" / "later.csv"),
                       "targets_path": str(tmp_path / "sub" / "later_targets.csv")}
    arr = np.genfromtxt(written["path"], delimiter=",", names=True, dtype=np.float64)
    assert list(arr.dtype.names) == MERGED_COLUMNS and arr.shape[0] == 30

    ctl.log_start()                                                  # the next session drops it
    with pytest.raises(RuntimeError):
        ctl.log_fetch()
    assert ctl.log_stop()["log_id"] == 2


def test_log_stop_can_leave_a_path_unwritten_for_log_save(tmp_path):
    view = _fresh_view()
    prod = _Producer(view)
    ctl = _StubController(view)
    ctl.log_start(tmp_path / "run.csv")
    prod.publish(12)
    summary = ctl.log_stop(save=False)
    assert summary["path"] is None and not (tmp_path / "run.csv").exists()
    assert ctl.log_save()["path"] == str(tmp_path / "run.csv")       # log_start's path
    assert (tmp_path / "run.csv").exists() and (tmp_path / "run_targets.csv").exists()


# --------------------------------------------------------------------------- daemon <-> client
def _daemon_on_loopback(controller):
    """A FrankaTwinDaemon with its __init__ bypassed (no osc_shm), bound to free
    loopback ports. Returns (daemon, cfg for a client)."""
    import zmq

    from frankatwin.daemon import SOCKET_LINGER_MS, FrankaTwinDaemon

    d = FrankaTwinDaemon.__new__(FrankaTwinDaemon)
    d.controller = controller
    d._stop_evt = threading.Event()
    d._op_lock = threading.Lock()
    d._pub_thread = d._watchdog_thread = d._gripper_thread = None
    ctx = zmq.Context.instance()
    d._rep, d._pub = ctx.socket(zmq.REP), ctx.socket(zmq.PUB)
    for sock in (d._rep, d._pub):
        sock.setsockopt(zmq.LINGER, SOCKET_LINGER_MS)
    cmd_port = d._rep.bind_to_random_port("tcp://127.0.0.1")
    state_port = d._pub.bind_to_random_port("tcp://127.0.0.1")
    cfg = types.SimpleNamespace(network=types.SimpleNamespace(
        nuc_host="127.0.0.1", cmd_port=cmd_port, state_port=state_port, state_cache=8))
    return d, cfg


def test_client_fetches_the_log_from_the_daemon_and_writes_it_here(tmp_path, monkeypatch):
    from frankatwin import daemon as daemon_mod
    from frankatwin.remote_client import FrankaTwinClient

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(daemon_mod, "LOG_FETCH_MAX_ROWS", 400)       # the daemon caps a chunk too
    view = _fresh_view()
    prod = _Producer(view)
    ctl = _StubController(view)
    d, cfg = _daemon_on_loopback(ctl)
    serve = threading.Thread(target=d.run, daemon=True)
    serve.start()
    try:
        with FrankaTwinClient(cfg) as robot:
            with pytest.raises(RuntimeError, match="no finished ring log"):
                robot.log_fetch()

            info = robot.log_start("data/run.csv")                   # relative: resolved here
            assert info["path"] == str(tmp_path / "data" / "run.csv") and info["log_id"] == 1
            assert (tmp_path / "data").is_dir()                      # created before the run
            rec = ctl._recorder
            assert rec.path is None                                  # the daemon has no file to write
            rec._stop_evt.set(); rec._thread.join(); rec._thread = None
            heads = []
            for k in range(25):                                      # 25 targets, 50 frames each
                heads.append(robot.set_ee_target(np.array([0.5, 0.01 * k, 0.4]), np.array([1.0, 0.0, 0.0, 0.0])))
                prod.publish(50); rec.poll()

            summary = robot.log_stop(save=False)
            assert summary["num_frames"] == 1250 and summary["path"] is None
            assert not (tmp_path / "data" / "run.csv").exists()

            # 1250 rows: 300-row requests, and a 1000-row one the daemon cuts to 400.
            merged = robot.log_fetch("merged", chunk_rows=300)
            np.testing.assert_array_equal(merged, ctl.log_fetch("merged"))
            np.testing.assert_array_equal(robot.log_fetch("merged", chunk_rows=1000), merged)
            np.testing.assert_array_equal(robot.log_fetch("targets"), ctl.log_fetch("targets"))
            assert merged.flags.writeable

            written = robot.log_save()
            assert written == {"path": str(tmp_path / "data" / "run.csv"),
                               "targets_path": str(tmp_path / "data" / "run_targets.csv")}
            arr = np.genfromtxt(written["path"], delimiter=",", names=True, dtype=np.float64)
            assert list(arr.dtype.names) == MERGED_COLUMNS and arr.shape[0] == 1250
            seq = arr["seq"].astype(int)
            for k in (0, 7, 24):
                rows = (seq > heads[k]) & (seq <= heads[k] + 50)
                np.testing.assert_allclose(arr["x_des_y"][rows], 0.01 * k)
            tv = np.genfromtxt(written["targets_path"], delimiter=",", names=True, dtype=np.float64)
            assert tv["head"].astype(int).tolist() == heads
            # The same bytes a recorder with a path writes on its own machine.
            ctl.log_save(tmp_path / "nuc.csv")
            assert (tmp_path / "nuc.csv").read_bytes() == (tmp_path / "data" / "run.csv").read_bytes()
            assert (tmp_path / "nuc_targets.csv").read_bytes() == (tmp_path / "data" / "run_targets.csv").read_bytes()

            # save=True (the default) stops, fetches and writes in one call.
            robot.log_start(tmp_path / "second.csv")
            ctl._recorder._stop_evt.set(); ctl._recorder._thread.join(); ctl._recorder._thread = None
            prod.publish(40)
            summary = robot.log_stop()
            assert summary["path"] == str(tmp_path / "second.csv") and summary["num_frames"] == 40
            assert (tmp_path / "second.csv").exists() and (tmp_path / "second_targets.csv").exists()

            # A client from before log_fetch names a file on the daemon's machine: refused.
            with pytest.raises(RuntimeError, match="takes no 'path'"):
                robot._call({"op": "log_start", "path": "data/old.csv"})
            assert not ctl.log_active
    finally:
        d.request_stop()
        serve.join(timeout=5.0)
    assert not serve.is_alive()
