"""examples/cart_impedance.py: what a run leaves on disk, with a stand-in daemon.

`--log-1khz` alone must be enough for a sysid run: the 1 kHz CSV, its targets
CSV and the sidecar all land on this machine under the name given, nothing is
asked of the daemon but the rows; no 50 Hz CSV unless `--log` asks for one. Runs
main() for 0.2 s against a fake client -- no robot, no daemon, no sockets.
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys
import types

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.local_controller import RobotState  # noqa: E402


def _load_cart_impedance():
    spec = importlib.util.spec_from_file_location("cart_impedance_example", ROOT / "examples" / "cart_impedance.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _FakeRobot:
    last: "_FakeRobot | None" = None

    fail_first_save = False        # class-level switch: the first log_save() raises OSError

    def __init__(self, _cfg):
        self.head = 1000
        self.log_path = None
        self.calls = []
        self.gains = None
        _FakeRobot.last = self

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return None

    def _state(self) -> RobotState:
        return RobotState(timestamp_s=self.head * 1e-3, q=np.zeros(7), dq=np.zeros(7),
                          ee_pos=np.array([0.31, 0.0, 0.48]), ee_quat=np.array([0.0, 1.0, 0.0, 0.0]),
                          tau=np.zeros(7), seq=self.head, tau_J=np.full(7, 1.0))

    def set_gains(self, **kw):
        self.gains = kw

    def wait_for_state(self, timeout_s=3.0):
        return self._state()

    def get_state(self):
        return self._state()

    def set_ee_target(self, pos, quat):
        self.calls.append("set_ee_target")
        self.head += 20
        return self.head

    # Same contract as FrankaTwinClient: the path is on this machine, log_stop
    # (save=False) only returns the summary, log_save writes the two CSVs.
    def log_start(self, path):
        self.calls.append("log_start")
        self.log_path = pathlib.Path(path).expanduser().resolve()
        return {"path": str(self.log_path), "seq_start": self.head, "poll_hz": 100.0, "log_id": 1}

    def log_stop(self, *, save=True):
        self.calls.append(f"log_stop(save={save})")
        return {"path": None, "targets_path": None, "num_frames": 200,
                "seq_start": 1000, "seq_first": 1001, "seq_last": 1200, "duration_s": 0.2,
                "num_targets": 10, "gaps": [], "dropped_frames": 0, "resets": 0, "poll_hz": 100.0,
                "log_id": 1}

    def log_save(self, path=None):
        self.calls.append("log_save")
        if _FakeRobot.fail_first_save:
            _FakeRobot.fail_first_save = False
            raise OSError("disk full")
        out = self.log_path if path is None else pathlib.Path(path)
        targets = out.with_name(out.stem + "_targets" + out.suffix)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text("seq\n")
        targets.write_text("target_idx\n")
        return {"path": str(out), "targets_path": str(targets)}


@pytest.fixture
def cart(monkeypatch, tmp_path):
    mod = _load_cart_impedance()
    monkeypatch.setattr(mod, "FrankaTwinClient", _FakeRobot)
    monkeypatch.setattr(mod, "load_config", lambda _p: types.SimpleNamespace(
        control=types.SimpleNamespace(kp_pos=200.0, kp_ori=20.0)))
    monkeypatch.setattr(mod.time, "sleep", lambda _s: None)
    monkeypatch.setattr(_FakeRobot, "fail_first_save", False)
    monkeypatch.setattr(_FakeRobot, "last", None)
    monkeypatch.chdir(tmp_path)
    return mod


def _run(mod, monkeypatch, *argv, rc=0):
    monkeypatch.setattr(sys, "argv", ["cart_impedance.py", *argv])
    assert mod.main() == rc
    return _FakeRobot.last


def test_log_1khz_alone_leaves_the_csv_pair_and_the_sidecar_on_this_machine(cart, monkeypatch, tmp_path):
    robot = _run(cart, monkeypatch, "--mode", "chirp", "--band", "high", "--duration", "0.2",
                 "--log-1khz", "data/sysid/run.csv")
    out = tmp_path / "data" / "sysid"
    assert robot.log_path == out / "run.csv"                         # relative: resolved here, not on the NUC
    assert sorted(f.name for f in out.iterdir()) == ["run.csv", "run.json", "run_targets.csv"]
    payload = json.loads((out / "run.json").read_text())
    # the pair the fit takes names itself; the summary carries the files' paths
    assert payload["csv_path"] == str(out / "run.csv") and payload["sidecar_path"] == str(out / "run.json")
    assert payload["ring_log"]["path"] == str(out / "run.csv")
    assert payload["ring_log"]["targets_path"] == str(out / "run_targets.csv")
    assert payload["ring_log"]["num_frames"] == 200
    assert payload["args"]["band"] == "high"                         # the band's defaults, no extra flags
    assert payload["args"]["amp_taper_exp"] == 0.5 and payload["args"]["ramp_down_s"] == 1.0
    assert len(payload["q_init"]) == 7 and payload["args"]["kp_pos"] == 200.0
    # stopped with the last setpoint, fetched only once the arm was sent back to the anchor
    calls = robot.calls
    assert calls[0] == "log_start" and calls.count("set_ee_target") == 11
    assert calls[-3:] == ["log_stop(save=False)", "set_ee_target", "log_save"]


def test_log_adds_the_50_hz_csv_and_a_sidecar_for_each(cart, monkeypatch, tmp_path):
    _run(cart, monkeypatch, "--mode", "chirp", "--duration", "0.2",
         "--log", "run50.csv", "--log-1khz", "khz/run.csv")
    assert (tmp_path / "run50.csv").exists() and (tmp_path / "khz" / "run.csv").exists()
    side50 = json.loads((tmp_path / "run50.json").read_text())      # --log decides where the sidecar goes
    side1k = json.loads((tmp_path / "khz" / "run.json").read_text())  # ...and the 1 kHz CSV gets its own
    assert side50["csv_path"] == str(tmp_path / "run50.csv")
    assert side1k["csv_path"] == str(tmp_path / "khz" / "run.csv")
    for side in (side50, side1k):
        assert side["args"]["band"] == "low" and side["args"]["ramp_down_s"] == 3.0
        assert side["ring_log"]["path"] == str(tmp_path / "khz" / "run.csv")
    assert side50["q_init"] == side1k["q_init"]


def test_both_logs_in_one_file_are_refused(cart, monkeypatch, tmp_path):
    for log in ("run.csv", "run_targets.csv"):
        assert _run(cart, monkeypatch, "--mode", "chirp", "--duration", "0.2",
                    "--log", log, "--log-1khz", "run.csv", rc=2) is None    # never connected
    assert list(tmp_path.iterdir()) == []


def test_unwritable_log_1khz_stops_before_the_run(cart, monkeypatch, tmp_path):
    assert _run(cart, monkeypatch, "--mode", "chirp", "--duration", "0.2",
                "--log-1khz", "/proc/frankatwin-no-such-dir/run.csv", rc=2) is None
    assert list(tmp_path.iterdir()) == []


def test_ring_log_falls_back_to_the_cwd_when_the_save_fails(cart, monkeypatch, tmp_path):
    monkeypatch.setattr(_FakeRobot, "fail_first_save", True)
    robot = _run(cart, monkeypatch, "--mode", "chirp", "--duration", "0.2", "--log-1khz", "deep/run.csv")
    assert robot.calls[-2:] == ["log_save", "log_save"]
    assert sorted(f.name for f in tmp_path.iterdir() if f.is_file()) == ["run.csv", "run.json", "run_targets.csv"]
    payload = json.loads((tmp_path / "run.json").read_text())
    assert payload["csv_path"] == str(tmp_path / "run.csv")
    assert not (tmp_path / "deep" / "run.json").exists()             # the sidecar follows the CSV


def test_a_failed_fetch_keeps_the_sidecar_and_says_so(cart, monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(_FakeRobot, "log_save",
                        lambda self, path=None: (_ for _ in ()).throw(TimeoutError("daemon did not reply")))
    _run(cart, monkeypatch, "--mode", "chirp", "--duration", "0.2", "--log-1khz", "run.csv")
    assert not (tmp_path / "run.csv").exists()
    payload = json.loads((tmp_path / "run.json").read_text())
    assert payload["csv_path"] is None and payload["ring_log"]["path"] is None
    assert "daemon did not reply" in payload["ring_log"]["save_error"]
    assert "FrankaTwinClient.log_save" in capsys.readouterr().err


def test_without_a_ring_log_nothing_is_asked_of_the_daemon(cart, monkeypatch, tmp_path):
    robot = _run(cart, monkeypatch, "--mode", "chirp", "--duration", "0.2", "--log", "only50.csv")
    assert robot.log_path is None and set(robot.calls) == {"set_ee_target"}
    payload = json.loads((tmp_path / "only50.json").read_text())
    assert payload["csv_path"] == str(tmp_path / "only50.csv") and payload["ring_log"] is None
