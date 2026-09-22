"""Validation step of the sysid workflow: replay -> figures + metrics.json.

`compare_sim_real.compute_metrics` is checked on a sim CSV built from the real
one with known offsets, so every RMSE has a closed form. `apply_sysid_params.py
--invoke-replay` is then run end to end with a stand-in for
`replay_python_csv_sim.py` (the real one needs Isaac Sim): one command has to
leave the sim CSV, the figures and metrics.json behind.
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys
import textwrap

import numpy as np
import pytest

pytest.importorskip("pandas")
pytest.importorskip("matplotlib")

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "isaaclab_sysid" / "scripts" / "tools"
sys.path.insert(0, str(ROOT / "python"))

from frankatwin.ring_log import MERGED_COLUMNS  # noqa: E402

POS_OFFSET = np.array([1.0e-3, -2.0e-3, 2.0e-3])          # sim - real [m]; norm = 3 mm
YAW_OFFSET = 0.01                                          # sim vs real, about z [rad]
JOINT_OFFSET = np.arange(1, 8) * 1.0e-3                    # sim - real [rad], j1..j7
FIGURES = ["joint_absolute_values_7x2.png", "joint_error_timeseries.png", "orientation_theta.png",
           "position_timeseries.png", "quaternion_timeseries.png", "traj3d.png"]


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _real_table(n: int = 500) -> np.ndarray:
    """A 1 kHz ring-log table: every joint sweeps 0.5 rad, the EE sits 5 mm off a moving target."""
    col = {c: i for i, c in enumerate(MERGED_COLUMNS)}
    t = np.arange(n) * 1.0e-3
    s = np.sin(2.0 * np.pi * 2.0 * t)
    table = np.zeros((n, len(MERGED_COLUMNS)))
    table[:, col["t_s"]] = t
    table[:, col["period_ms"]] = 1.0
    for j in range(1, 8):
        table[:, col[f"q{j}"]] = 0.25 * s                   # max - min = 0.5 rad
        table[:, col[f"dq{j}"]] = 0.25 * 2.0 * np.pi * 2.0 * np.cos(2.0 * np.pi * 2.0 * t)
    for k, axis in enumerate("xyz"):
        table[:, col[f"x_des_{axis}"]] = 0.4 + 0.05 * s
        table[:, col[f"x_{axis}"]] = 0.4 + 0.05 * s + (5.0e-3 if k == 0 else 0.0)
    table[:, col["quat_w"]] = table[:, col["quat_des_w"]] = 1.0
    table[:, col["seq"]] = np.arange(n) + 1
    return table


def _write_pair(tmp_path: pathlib.Path):
    """real.csv (ring-log columns) and a sim CSV that is real + the known offsets."""
    col = {c: i for i, c in enumerate(MERGED_COLUMNS)}
    real = _real_table()
    sim = real.copy()
    for k, axis in enumerate("xyz"):
        sim[:, col[f"x_{axis}"]] += POS_OFFSET[k]
    for j in range(1, 8):
        sim[:, col[f"q{j}"]] += JOINT_OFFSET[j - 1]
        sim[:, col[f"dq{j}"]] += 10.0 * JOINT_OFFSET[j - 1]
    sim[:, col["quat_z"]], sim[:, col["quat_w"]] = np.sin(YAW_OFFSET / 2.0), np.cos(YAW_OFFSET / 2.0)
    real_csv, sim_csv = tmp_path / "run.csv", tmp_path / "run_sim_sysid.csv"
    for path, table in ((real_csv, real), (sim_csv, sim)):
        np.savetxt(path, table, delimiter=",", header=",".join(MERGED_COLUMNS), comments="", fmt="%.9f")
    return real_csv, sim_csv


def _check_metrics(m: dict) -> None:
    pos = m["ee"]["pos"]
    assert [pos["rmse_m"][a] for a in "xyz"] == pytest.approx(np.abs(POS_OFFSET), rel=1e-6)
    assert pos["rmse_m"]["3d"] == pytest.approx(3.0e-3, rel=1e-6)
    assert pos["max_abs_m"]["3d"] == pytest.approx(3.0e-3, rel=1e-6)
    assert m["ee"]["ori"]["rmse_rad"] == pytest.approx(YAW_OFFSET, rel=1e-4)
    assert m["ee"]["ori"]["max_rad"] == pytest.approx(YAW_OFFSET, rel=1e-4)

    j = m["joints"]
    assert j["pos_rmse_rad"] == pytest.approx(JOINT_OFFSET, rel=1e-6)
    assert j["pos_mse_all_rad2"] == pytest.approx(np.mean(JOINT_OFFSET**2), rel=1e-6)
    assert j["pos_rmse_all_rad"] == pytest.approx(np.sqrt(np.mean(JOINT_OFFSET**2)), rel=1e-6)
    assert j["real_motion_range_rad"] == pytest.approx([0.5] * 7, rel=1e-3)
    assert j["pos_rmse_pct_of_motion"] == pytest.approx(100.0 * JOINT_OFFSET / 0.5, rel=1e-3)
    assert j["vel_rmse_rad_s"] == pytest.approx(10.0 * JOINT_OFFSET, rel=1e-6)

    # tracking: each side against the target, not the sim-to-real gap
    track = m["tracking"]
    assert track["real"]["pos"]["rmse_m"]["x"] == pytest.approx(5.0e-3, rel=1e-6)
    assert track["sim"]["pos"]["rmse_m"]["x"] == pytest.approx(6.0e-3, rel=1e-6)
    assert track["real"]["ori"]["rmse_rad"] == pytest.approx(0.0, abs=1e-6)
    assert track["sim"]["ori"]["rmse_rad"] == pytest.approx(YAW_OFFSET, rel=1e-4)
    assert m["num_samples"] == 500 and m["duration_s"] == pytest.approx(0.499)


def test_compare_scores_the_pair_and_writes_figures_and_metrics(tmp_path, capsys):
    cmp = _load("compare_sim_real")
    real_csv, sim_csv = _write_pair(tmp_path)
    m = cmp.compare(real_csv, sim_csv)
    _check_metrics(m)

    out = tmp_path / "compare_run_sim_sysid"                         # default: next to the sim CSV
    assert m["out_dir"] == str(out) and m["metrics_path"] == str(out / "metrics.json")
    assert sorted(f.name for f in out.iterdir()) == sorted(FIGURES + ["metrics.json"])
    on_disk = json.loads((out / "metrics.json").read_text())
    _check_metrics(on_disk)
    assert on_disk["schema_version"] == 1
    assert on_disk["real_csv"] == str(real_csv) and on_disk["sim_csv"] == str(sim_csv)

    printed = capsys.readouterr().out                                # the table holds the same numbers
    assert "err_pos_rms_3d [mm]" in printed and "3.0000" in printed
    assert "rmse [mrad]" in printed and "joint-position MSE [rad^2]: 2.0000e-05" in printed


def test_compare_without_save_writes_nothing(tmp_path):
    cmp = _load("compare_sim_real")
    real_csv, sim_csv = _write_pair(tmp_path)
    m = cmp.compare(real_csv, sim_csv, save=False)
    _check_metrics(m)
    assert m["out_dir"] is None and m["metrics_path"] is None
    assert sorted(f.name for f in tmp_path.iterdir()) == ["run.csv", "run_sim_sysid.csv"]


def test_sim_on_another_time_base_is_interpolated_onto_the_real_one(tmp_path):
    cmp = _load("compare_sim_real")
    real_csv, sim_csv = _write_pair(tmp_path)
    table = np.genfromtxt(sim_csv, delimiter=",", skip_header=1)
    np.savetxt(sim_csv, table[::2], delimiter=",", header=",".join(MERGED_COLUMNS), comments="", fmt="%.9f")
    m = cmp.compare(real_csv, sim_csv, save=False)
    assert m["num_samples"] == 500                                   # scored on the real rows
    assert m["ee"]["pos"]["rmse_m"]["3d"] == pytest.approx(3.0e-3, rel=1e-2)
    assert m["joints"]["pos_rmse_rad"] == pytest.approx(JOINT_OFFSET, rel=5e-2)


# --------------------------------------------------------------------------- one command
FAKE_REPLAY = textwrap.dedent('''
    """Stand-in for replay_python_csv_sim.py: sim = real + known offsets."""
    import argparse, json, sys
    import numpy as np
    p = argparse.ArgumentParser()
    for flag in ("--real-csv", "--real-sidecar", "--out-csv", "--out-sidecar", "--gain-source",
                 "--control-mode", "--warmup-steps", "--sysid-params"):
        p.add_argument(flag)
    p.add_argument("--headless", action="store_true")
    a = p.parse_args()
    header = open(a.real_csv).readline().strip()
    col = {c: i for i, c in enumerate(header.split(","))}
    t = np.genfromtxt(a.real_csv, delimiter=",", skip_header=1)
    for k, axis in enumerate("xyz"):
        t[:, col[f"x_{axis}"]] += %(pos)s[k]
    for j in range(1, 8):
        t[:, col[f"q{j}"]] += %(joint)s[j - 1]
        t[:, col[f"dq{j}"]] += 10.0 * %(joint)s[j - 1]
    t[:, col["quat_z"]], t[:, col["quat_w"]] = np.sin(%(yaw)s / 2.0), np.cos(%(yaw)s / 2.0)
    np.savetxt(a.out_csv, t, delimiter=",", header=header, comments="", fmt="%%.9f")
    json.dump({"controller": "fake_replay", "argv": sys.argv[1:]}, open(a.out_sidecar, "w"))
''') % {"pos": POS_OFFSET.tolist(), "joint": JOINT_OFFSET.tolist(), "yaw": YAW_OFFSET}


@pytest.fixture
def validation_inputs(tmp_path):
    real_csv, sim_csv = _write_pair(tmp_path)
    sim_csv.unlink()                                                 # the replay has to produce it
    sidecar = tmp_path / "run.json"
    sidecar.write_text(json.dumps({"controller": "python_multiband_excitation",
                                   "args": {"kp_pos": 500.0, "kp_ori": 30.0}}))
    best = tmp_path / "sysid_best_params.json"
    best.write_text(json.dumps({"best_params_decoded": {
        "armature": [0.1] * 7, "mu_static": [0.5] * 7, "dynamic_ratio": [0.5] * 7,
        "mu_viscous": [1.0] * 7, "motor_delay_steps": 1}}))
    replay = tmp_path / "fake_replay.py"
    replay.write_text(FAKE_REPLAY)
    return real_csv, sidecar, best, replay


def _apply(monkeypatch, *argv):
    mod = _load("apply_sysid_params")
    monkeypatch.setattr(sys, "argv", ["apply_sysid_params.py", *map(str, argv)])
    mod.main()


def test_invoke_replay_leaves_the_sim_csv_the_figures_and_the_metrics(validation_inputs, tmp_path, monkeypatch, capsys):
    real_csv, sidecar, best, replay = validation_inputs
    _apply(monkeypatch, "--best", best, "--invoke-replay", "--replay-script", replay,
           "--real-csv", real_csv, "--real-sidecar", sidecar, "--headless")

    assert (tmp_path / "run_sim_sysid.csv").exists() and (tmp_path / "run_sim_sysid.json").exists()
    replay_argv = json.loads((tmp_path / "run_sim_sysid.json").read_text())["argv"]
    assert replay_argv[replay_argv.index("--sysid-params") + 1] == str(best) and "--headless" in replay_argv

    out = tmp_path / "compare_run_sim_sysid"
    assert sorted(f.name for f in out.iterdir()) == sorted(FIGURES + ["metrics.json"])
    m = json.loads((out / "metrics.json").read_text())
    _check_metrics(m)
    assert m["sysid_params"] == str(best)                            # which parameters were scored
    assert m["real_sidecar"] == str(sidecar) and m["sim_sidecar"] == str(tmp_path / "run_sim_sysid.json")
    printed = capsys.readouterr().out
    assert "EE position RMSE 3.00 mm (3-D)" in printed and "EE orientation RMSE 10.00 mrad" in printed


def test_no_compare_stops_after_the_replay_and_compare_dir_moves_the_output(validation_inputs, tmp_path, monkeypatch):
    real_csv, sidecar, best, replay = validation_inputs
    common = ("--best", best, "--invoke-replay", "--replay-script", replay,
              "--real-csv", real_csv, "--real-sidecar", sidecar)
    _apply(monkeypatch, *common, "--no-compare")
    assert (tmp_path / "run_sim_sysid.csv").exists()
    assert not (tmp_path / "compare_run_sim_sysid").exists()

    _apply(monkeypatch, *common, "--compare-dir", tmp_path / "report")
    assert (tmp_path / "report" / "metrics.json").exists() and (tmp_path / "report" / "traj3d.png").exists()
    assert not (tmp_path / "compare_run_sim_sysid").exists()
