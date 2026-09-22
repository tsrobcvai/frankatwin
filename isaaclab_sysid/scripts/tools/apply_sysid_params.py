# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Apply fitted sysid parameters: replay a real run in sim and score the result.

``--invoke-replay`` is the whole validation step in one command. It runs
``replay_python_csv_sim.py`` with the parameters applied (a subprocess: that one
needs Isaac Sim), then hands the real CSV and the sim CSV it wrote to
``compare_sim_real.compare`` (numpy / pandas / matplotlib only), which prints the
scores and writes the figures and ``metrics.json`` -- EE and per-joint RMSE of
sim against real -- to ``<sim-csv-dir>/compare_<sim-stem>/``. ``--no-compare``
stops after the replay; ``compare_sim_real.py`` on its own redoes the figures
without another replay.

``--print-snippet`` prints the parameters as a config snippet instead.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Apply Franka sysid params and optionally replay.")
    parser.add_argument("--best", required=True, help="Path to sysid_best_params.json")
    parser.add_argument("--print-snippet", action="store_true", help="Print a config snippet for manual patching.")
    parser.add_argument("--invoke-replay", action="store_true",
                        help="Replay --real-csv in sim with the sysid params applied, then compare sim against "
                             "real: figures + metrics.json (EE and per-joint RMSE).")
    parser.add_argument("--python-exe", default=sys.executable, help="Python executable used for --invoke-replay.")
    parser.add_argument(
        "--replay-script",
        default=str(Path(__file__).resolve().parent / "replay_python_csv_sim.py"),
        help="Path to replay_python_csv_sim.py",
    )
    parser.add_argument("--real-csv", default=None, help="Required with --invoke-replay")
    parser.add_argument("--real-sidecar", default=None, help="Required with --invoke-replay")
    parser.add_argument("--out-csv", default=None, help="Default: <real>_sim_sysid.csv")
    parser.add_argument("--out-sidecar", default=None, help="Default: <real>_sim_sysid.json")
    parser.add_argument("--gain-source", default="sidecar", choices=["sidecar", "env_cfg"])
    parser.add_argument("--control-mode", default="task_impedance", choices=["task_impedance", "osc"])
    parser.add_argument("--warmup-steps", type=int, default=50)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-compare", action="store_true",
                        help="With --invoke-replay: stop after the replay, no figures and no metrics.json.")
    parser.add_argument("--compare-dir", default=None,
                        help="Directory for the figures and metrics.json. Default: <out-csv-dir>/compare_<out-csv-stem>.")
    parser.add_argument("--show", action="store_true", help="Also display the comparison figures.")
    parser.add_argument("--dpi", type=int, default=140)
    return parser.parse_args()


def load_best(path: Path) -> dict:
    payload = json.loads(path.read_text())
    decoded = payload.get("best_params_decoded", payload)
    required = ["armature", "mu_static", "dynamic_ratio", "mu_viscous", "motor_delay_steps"]
    missing = [k for k in required if k not in decoded]
    if missing:
        raise KeyError(f"{path} missing required keys: {missing}")
    return decoded


def print_snippet(best: dict):
    print("# Paste into replay/sysid cfg if you want static defaults:")
    print("sysid_best = {")
    print(f"    'armature': {best['armature']},")
    print(f"    'mu_static': {best['mu_static']},")
    print(f"    'dynamic_ratio': {best['dynamic_ratio']},")
    print(f"    'mu_viscous': {best['mu_viscous']},")
    print(f"    'motor_delay_steps': {int(best['motor_delay_steps'])},")
    print("}")


def default_outputs(real_csv_path: Path, out_csv: str | None, out_sidecar: str | None) -> tuple[Path, Path]:
    if out_csv is None:
        out_csv_path = real_csv_path.with_name(f"{real_csv_path.stem}_sim_sysid.csv")
    else:
        out_csv_path = Path(out_csv)
    if out_sidecar is None:
        out_sidecar_path = real_csv_path.with_name(f"{real_csv_path.stem}_sim_sysid.json")
    else:
        out_sidecar_path = Path(out_sidecar)
    return out_csv_path.resolve(), out_sidecar_path.resolve()


def invoke_replay(args: argparse.Namespace, best_path: Path) -> tuple[Path, Path, Path, Path]:
    """Run the replay. Returns (real_csv, real_sidecar, out_csv, out_sidecar)."""
    if args.real_csv is None or args.real_sidecar is None:
        raise ValueError("--real-csv and --real-sidecar are required with --invoke-replay")

    real_csv = Path(args.real_csv).expanduser().resolve()
    real_sidecar = Path(args.real_sidecar).expanduser().resolve()
    out_csv, out_sidecar = default_outputs(real_csv, args.out_csv, args.out_sidecar)
    cmd = [
        args.python_exe,
        str(Path(args.replay_script).expanduser().resolve()),
        "--real-csv",
        str(real_csv),
        "--real-sidecar",
        str(real_sidecar),
        "--out-csv",
        str(out_csv),
        "--out-sidecar",
        str(out_sidecar),
        "--gain-source",
        str(args.gain_source),
        "--control-mode",
        str(args.control_mode),
        "--warmup-steps",
        str(args.warmup_steps),
        "--sysid-params",
        str(best_path),
    ]
    if args.headless:
        cmd.append("--headless")

    print("[apply_sysid] running replay command:")
    print(" ".join(cmd))
    subprocess.run(cmd, check=True)
    print(f"[apply_sysid] wrote {out_csv}")
    print(f"[apply_sysid] wrote {out_sidecar}")
    return real_csv, real_sidecar, out_csv, out_sidecar


def invoke_compare(args: argparse.Namespace, best_path: Path, real_csv: Path, real_sidecar: Path,
                   sim_csv: Path, sim_sidecar: Path) -> dict:
    """Score the sim CSV against the real one: figures + metrics.json."""
    # compare_sim_real.py sits next to this script (scripts/tools/ once installed).
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    try:
        from compare_sim_real import compare
    except ImportError as e:
        raise SystemExit(
            f"[apply_sysid] the replay is written, but the comparison cannot run: {e}\n"
            "[apply_sysid] pip install pandas matplotlib in this env, then:\n"
            f"  python {Path(__file__).resolve().parent / 'compare_sim_real.py'} "
            f"--real-csv {real_csv} --sim-csv {sim_csv} --save"
        ) from e
    metrics = compare(
        real_csv, sim_csv,
        real_sidecar=real_sidecar, sim_sidecar=sim_sidecar if sim_sidecar.is_file() else None,
        out_dir=args.compare_dir, save=True, show=args.show, dpi=args.dpi,
        extra={"sysid_params": str(best_path)},
    )
    ee, joints = metrics["ee"], metrics["joints"]
    print(f"[apply_sysid] sim vs real: EE position RMSE {ee['pos']['rmse_m']['3d'] * 1000:.2f} mm (3-D), "
          f"EE orientation RMSE {ee['ori']['rmse_rad'] * 1000:.2f} mrad"
          + (f", joint RMSE {joints['pos_rmse_all_rad'] * 1000:.2f} mrad (all joints)" if joints else ""))
    print(f"[apply_sysid] wrote {metrics['metrics_path']}")
    return metrics


def main():
    args = parse_args()
    best_path = Path(args.best).expanduser().resolve()
    if not best_path.is_file():
        raise FileNotFoundError(best_path)
    best = load_best(best_path)

    if not args.print_snippet and not args.invoke_replay:
        raise ValueError("At least one of --print-snippet or --invoke-replay must be set.")

    if args.print_snippet:
        print_snippet(best)
    if args.invoke_replay:
        real_csv, real_sidecar, out_csv, out_sidecar = invoke_replay(args, best_path)
        if not args.no_compare:
            invoke_compare(args, best_path, real_csv, real_sidecar, out_csv, out_sidecar)


if __name__ == "__main__":
    main()
