#!/usr/bin/env python3
"""Record the 1 kHz state ring on the NUC without going through the daemon.

Opens the existing shm segment read-only and drains every frame into a CSV
with the ``cart_impedance.py --log`` columns (+ ``seq``, ``target_idx``), see
``frankatwin.ring_log``. Setpoints are taken from the command block, polled
at ``--poll-hz``: a change is stamped with the head seen at the poll, so its
``seq_effective`` is late by up to one poll period (10 ms at 100 Hz). For
tick-exact setpoints record through the daemon instead
(``robot.log_start()`` / ``examples/cart_impedance.py --log-1khz``), where
the writer stamps each target itself.

Run on the NUC while osc_shm is up (daemon or by hand):
  python scripts/shm_log.py --out data/run_1khz.csv --duration 30
Ctrl-C stops early and still writes the file.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from frankatwin.ring_log import DEFAULT_POLL_HZ, RunRecorder  # noqa: E402
from frankatwin.shm_layout import FRANKATWIN_SHM_DEFAULT_NAME, SharedMemoryAccess  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Dump the 1 kHz shm state ring to CSV (NUC side).")
    p.add_argument("--out", type=str, required=True, help="Output CSV; <stem>_targets.csv is written next to it.")
    p.add_argument("--duration", type=float, default=None, help="Seconds to record (default: until Ctrl-C).")
    p.add_argument("--poll-hz", type=float, default=DEFAULT_POLL_HZ,
                   help="Ring poll / command-block watch rate [Hz] (default: 100; the ring holds 1 s).")
    p.add_argument("--shm-name", type=str, default=FRANKATWIN_SHM_DEFAULT_NAME, help="POSIX shm name.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    shm = SharedMemoryAccess(args.shm_name, create=False)
    view = shm.view
    if view.state_head == 0:
        print(f"[shm_log] no frames in {args.shm_name} yet -- is osc_shm running?", file=sys.stderr)
        shm.close()
        return 1

    rec = RunRecorder(view, args.out, poll_hz=args.poll_hz)
    rec.start()
    print(f"[shm_log] recording from seq {rec.seq_start} at {args.poll_hz:g} Hz polls -> {rec.path}")

    period = 1.0 / float(args.poll_hz)
    t_end = None if args.duration is None else time.monotonic() + float(args.duration)
    last = view.read_command()
    try:
        while t_end is None or time.monotonic() < t_end:
            time.sleep(period)
            cmd = view.read_command()
            if int(cmd["seq"][0]) != int(last["seq"][0]) and (
                not np.array_equal(cmd["target_pos"][0], last["target_pos"][0])
                or not np.array_equal(cmd["target_quat"][0], last["target_quat"][0])
            ):
                rec.record_target(view.state_head, cmd["target_pos"][0], cmd["target_quat"][0])
            last = cmd
    except KeyboardInterrupt:
        print("[shm_log] interrupted, writing", file=sys.stderr)
    finally:
        summary = rec.stop()
        # Drop every view of the buffer before closing, or SharedMemory refuses
        # ("cannot close exported pointers exist") and complains again at exit.
        del rec, cmd, last
        shm.view = None
        view = None
        shm.close(unlink=False)

    print(
        f"[shm_log] wrote {summary['path']}: {summary['num_frames']} frames "
        f"(seq {summary['seq_first']}..{summary['seq_last']}, {summary['duration_s']:.3f} s), "
        f"{summary['num_targets']} targets (poll-resolution), "
        f"{summary['dropped_frames']} dropped, {len(summary['gaps'])} gaps, {summary['resets']} resets"
    )
    if summary["dropped_frames"] or summary["resets"]:
        print("[shm_log] WARNING: frames were lost or the controller restarted; "
              "the log is not a clean 1 kHz run", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
