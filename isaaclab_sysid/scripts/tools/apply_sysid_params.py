# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Utility for applying sysid best parameters to replay runs."""

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
    parser.add_argument("--invoke-replay", action="store_true", help="Invoke replay_real_step5b_sim.py with sysid params.")
    parser.add_argument("--python-exe", default=sys.executable, help="Python executable used for --invoke-replay.")
    parser.add_argument(
        "--replay-script",
        default=str(Path(__file__).resolve().parent / "replay_real_step5b_sim.py"),
        help="Path to replay_real_step5b_sim.py",
    )
    parser.add_argument("--real-csv", default=None, help="Required with --invoke-replay")
    parser.add_argument("--real-sidecar", default=None, help="Required with --invoke-replay")
    parser.add_argument("--out-csv", default=None, help="Default: <real>_sim_sysid.csv")
    parser.add_argument("--out-sidecar", default=None, help="Default: <real>_sim_sysid.json")
    parser.add_argument("--gain-source", default="sidecar", choices=["sidecar", "env_cfg"])
    parser.add_argument("--control-mode", default="task_impedance", choices=["task_impedance", "osc"])
    parser.add_argument("--warmup-steps", type=int, default=50)
    parser.add_argument("--headless", action="store_true")
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


def invoke_replay(args: argparse.Namespace, best_path: Path):
    if args.real_csv is None or args.real_sidecar is None:
        raise ValueError("--real-csv and --real-sidecar are required with --invoke-replay")

    real_csv = Path(args.real_csv).expanduser().resolve()
    out_csv, out_sidecar = default_outputs(real_csv, args.out_csv, args.out_sidecar)
    cmd = [
        args.python_exe,
        str(Path(args.replay_script).expanduser().resolve()),
        "--real-csv",
        str(real_csv),
        "--real-sidecar",
        str(Path(args.real_sidecar).expanduser().resolve()),
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
        invoke_replay(args, best_path)


if __name__ == "__main__":
    main()
