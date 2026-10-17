#!/usr/bin/env python3
"""Run a local command or srun allocation with a persistent, independent monitor.

Usage: python scripts/run_monitored.py --run-dir /absolute/new/run -- COMMAND ...
For Slurm, put this script's --worker invocation after the srun options; see README.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time


def now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def write(path, data):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    tmp.replace(path)


def alive(pid):
    try:
        os.kill(pid, 0)
        stat = Path(f"/proc/{pid}/stat")
        return not stat.exists() or stat.read_text().split(") ")[1][0] != "Z"
    except (OSError, IndexError):
        return False


def monitor(root, interval):
    plan = json.loads((root / "command.json").read_text())
    pid = int((root / "launcher.pid").read_text())
    previous_sizes, unchanged_since = {}, time.time()
    while True:
        record = {
            "utc": now(),
            "launcher_pid": pid,
            "launcher_alive": alive(pid),
            "command": plan["command"],
            "cwd": plan["cwd"],
            "logs": {},
            "possible_errors": [],
        }
        sizes = {}
        for name in ("stdout.log", "stderr.log"):
            path = root / name
            if path.exists():
                sizes[name] = path.stat().st_size
                with path.open("rb") as stream:
                    stream.seek(max(0, sizes[name] - 24000))
                    tail = stream.read().decode(errors="replace").splitlines()
                record["logs"][name] = {
                    "path": str(path),
                    "bytes": sizes[name],
                    "tail": tail[-12:],
                }
                record["possible_errors"] += [
                    line
                    for line in tail
                    if re.search(
                        r"Traceback|\b(?:NaN|Inf)\b|buffer.*overflow|dropped.*contact|CUDA error|segmentation fault|\bFAILED\b|\bERROR\b",
                        line,
                        re.IGNORECASE,
                    )
                ]
                if name == "stderr.log" and not (root / "job_id").exists():
                    matches = re.findall(r"srun: job (\d+)", "\n".join(tail))
                    if matches:
                        (root / "job_id").write_text(matches[-1])
        if sizes != previous_sizes:
            unchanged_since = time.time()
        previous_sizes = sizes
        record["seconds_without_log_progress"] = round(time.time() - unchanged_since, 1)
        for name in ("process.pid", "worker.pid", "job_id"):
            if (root / name).exists():
                record[name] = (root / name).read_text().strip()
        if "job_id" in record:
            try:
                result = subprocess.run(
                    ["squeue", "-h", "-j", record["job_id"], "-o", "%i %T %M %R"],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                record["slurm"] = result.stdout.strip()
            except (OSError, subprocess.TimeoutExpired) as exc:
                record["slurm_query_error"] = str(exc)
        done = (root / "exit_code").exists()
        if done:
            record["exit_code"] = int((root / "exit_code").read_text())
            record["state"] = "terminal"
            record["result_files"] = [
                str(p)
                for p in root.rglob("*")
                if p.is_file() and p.suffix in (".json", ".csv", ".npz", ".png", ".mp4")
            ]
        elif not record["launcher_alive"]:
            record["state"] = "launcher_missing_exit_status_unknown"
        else:
            record["state"] = "running"
        write(root / "status.json", record)
        with (root / "monitor.jsonl").open("a") as stream:
            stream.write(json.dumps(record) + "\n")
        if done or not record["launcher_alive"]:
            return
        time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=60.0)
    parser.add_argument(
        "--mode", choices=("launch", "supervise", "monitor", "worker"), default="launch"
    )
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    root = args.run_dir.resolve()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if args.mode == "monitor":
        monitor(root, args.interval)
        return
    if args.mode == "worker":
        (root / "worker.pid").write_text(str(os.getpid()))
        if os.environ.get("SLURM_JOB_ID"):
            (root / "job_id").write_text(os.environ["SLURM_JOB_ID"])
        write(
            root / "worker_command.json",
            {"command": command, "utc": now(), "hostname": os.uname().nodename},
        )
        os.execvp(command[0], command)
    if args.mode == "supervise":
        plan = json.loads((root / "command.json").read_text())
        while not (root / "status.json").exists():
            time.sleep(0.05)
        code = 1
        try:
            child = subprocess.Popen(
                plan["command"], cwd=plan["cwd"], stdin=subprocess.DEVNULL
            )
            (root / "process.pid").write_text(str(child.pid))
            code = child.wait()
        finally:
            (root / "exit_code").write_text(str(code))
        return
    if not command or args.interval <= 0:
        parser.error("a command and positive interval are required")
    root.mkdir(parents=True, exist_ok=False)
    write(
        root / "command.json",
        {"command": command, "cwd": str(Path.cwd()), "submitted_utc": now()},
    )
    script = str(Path(__file__).resolve())
    with (root / "stdout.log").open("w") as out, (root / "stderr.log").open("w") as err:
        launcher = subprocess.Popen(
            [sys.executable, script, "--run-dir", str(root), "--mode", "supervise"],
            stdin=subprocess.DEVNULL,
            stdout=out,
            stderr=err,
            start_new_session=True,
        )
    (root / "launcher.pid").write_text(str(launcher.pid))
    with (root / "monitor.stdout").open("w") as out, (root / "monitor.stderr").open(
        "w"
    ) as err:
        watcher = subprocess.Popen(
            [
                sys.executable,
                script,
                "--run-dir",
                str(root),
                "--mode",
                "monitor",
                "--interval",
                str(args.interval),
            ],
            stdin=subprocess.DEVNULL,
            stdout=out,
            stderr=err,
            start_new_session=True,
        )
    (root / "monitor.pid").write_text(str(watcher.pid))
    for _ in range(100):
        if watcher.poll() is not None:
            raise RuntimeError(f"monitor exited during startup; inspect {root}")
        if (root / "status.json").exists():
            print(
                json.dumps(
                    {
                        "run_dir": str(root),
                        "launcher_pid": launcher.pid,
                        "monitor_pid": watcher.pid,
                        "first_status_verified": True,
                    }
                )
            )
            return
        time.sleep(0.05)
    raise RuntimeError(f"monitor did not produce its first status: {root}")


if __name__ == "__main__":
    main()
