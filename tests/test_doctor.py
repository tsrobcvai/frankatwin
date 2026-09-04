"""python -m frankatwin.doctor degrades gracefully (no daemon, bad config)."""

from __future__ import annotations

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.doctor import main as doctor_main  # noqa: E402

def test_doctor_pc_without_daemon_reports_and_exits_1(monkeypatch, tmp_path, capsys):
    # Point at an unroutable daemon so the check fails fast instead of hanging.
    cfg = (ROOT / "config" / "robot.yaml").read_text().replace('nuc_host: "172.16.0.1"', 'nuc_host: "127.0.0.1"')
    cfg = cfg.replace("cmd_port: 5555", "cmd_port: 1").replace("state_port: 5556", "state_port: 2")
    path = tmp_path / "robot.yaml"
    path.write_text(cfg)
    rc = doctor_main(["--config", str(path), "--role", "pc"])
    out = capsys.readouterr().out
    assert rc == 1
    assert "[ok] config" in out and "[XX] daemon" in out
    assert "problem(s)" in out


def test_doctor_bad_config_is_reported_not_raised(tmp_path, capsys):
    path = tmp_path / "robot.yaml"
    path.write_text("network: {nuc_host: x, cmd_port: 1, state_port: 1}\n")  # ports collide
    rc = doctor_main(["--config", str(path)])
    out = capsys.readouterr().out
    assert rc == 1 and "[XX] config" in out

