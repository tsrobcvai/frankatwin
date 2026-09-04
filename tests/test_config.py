"""Config resolution: repo default, explicit path, env override, build_dir."""

from __future__ import annotations

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.config import DEFAULT_CONFIG_PATH, load_config, resolve_config_path  # noqa: E402


def test_default_is_repo_config(monkeypatch, tmp_path):
    monkeypatch.delenv("FRANKATWIN_CONFIG", raising=False)
    monkeypatch.chdir(tmp_path)                         # cwd must not matter
    cfg = load_config()
    assert cfg.source_path == DEFAULT_CONFIG_PATH == ROOT / "config" / "robot.yaml"
    assert cfg.paths.build_dir == ROOT / "build"        # relative build_dir -> repo root
    assert cfg.paths.shm_name.startswith("/")
    assert cfg.control.kd_pos_effective == pytest.approx(2 * cfg.control.kp_pos ** 0.5)


def test_env_override(monkeypatch, tmp_path):
    alt = tmp_path / "alt.yaml"
    alt.write_text(DEFAULT_CONFIG_PATH.read_text().replace('build_dir: "build"', 'build_dir: "/opt/ft/build"'))
    monkeypatch.setenv("FRANKATWIN_CONFIG", str(alt))
    assert resolve_config_path() == alt.resolve()
    cfg = load_config()
    assert cfg.paths.build_dir == pathlib.Path("/opt/ft/build")


def test_explicit_path_beats_env(monkeypatch, tmp_path):
    monkeypatch.setenv("FRANKATWIN_CONFIG", "/nonexistent/robot.yaml")
    cfg = load_config(DEFAULT_CONFIG_PATH)
    assert cfg.source_path == DEFAULT_CONFIG_PATH


def test_missing_file_message(monkeypatch):
    monkeypatch.setenv("FRANKATWIN_CONFIG", "/nonexistent/robot.yaml")
    with pytest.raises(FileNotFoundError, match="FRANKATWIN_CONFIG"):
        load_config()
