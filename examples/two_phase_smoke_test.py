"""Minimal 2-phase smoke test — debugging multi-phase chase on real arm.

This is a near-verbatim extension of ``fixed_delta_pose_test.py`` (which we
have repeatedly verified runs cleanly on the real arm).  The ONLY structural
difference is the outer loop runs TWO phases sequentially instead of one:

    Phase 1: 2 s, δ_pos = [+0.015, 0, 0]   — push +x
    Phase 2: 2 s, δ_pos = [0, +0.015, 0]   — push +y

Critically, this script keeps everything else identical to fixed_delta:

  * orientation target is FROZEN at the startup quat (no rotation chase)
  * uses ``robot.get_state()`` (SUB cache), NOT ``fresh=True``
  * settle period between phases is OPT-IN via ``--settle`` (default 0.3 s)
  * no rotation phases, no rotation deltas in the action

Purpose: if THIS works, the multi-phase chase pattern itself is fine on real,
which means the bug in ``six_dof_pose_test.py`` is one of the extras
(orientation chase, settle period, rotation phases).  If this DOES NOT work,
the bug is more fundamental — likely an osc_shm / daemon version mismatch
between PC client and NUC binaries — and we should check the NUC repo before
adding any more complexity.

A/B test for the "phase-boundary discontinuity → REFLEX" hypothesis
-------------------------------------------------------------------
``--settle`` inserts a zero-delta hold phase between the two active phases so
the arm's velocity damps to ~0 before the commanded direction changes (the
same fix six_dof_pose_test.py already relies on).  Run both:

    --settle 0      reproduces the failure (sharp +x -> +y reversal trips the
                    libfranka jerk/torque-discontinuity reflex; osc_shm dies
                    mid-run and phase 2 never moves)
    --settle 0.3    velocity damps to ~0 at the boundary; if this completes
                    both phases cleanly (phase 2 shows a real +y increment),
                    the boundary discontinuity is confirmed as the cause.

Run::

    cd /home/tao/Projects/panda_control
    # test the fix (settle on):
    python examples/two_phase_smoke_test.py \\
        --kp-pos 500 --kp-ori 30 --settle 0.3 \\
        --log data/smoke_2phase_$(date +%Y%m%d_%H%M%S).csv
    # reproduce the failure (settle off):
    python examples/two_phase_smoke_test.py \\
        --kp-pos 500 --kp-ori 30 --settle 0 \\
        --log data/smoke_2phase_nosettle_$(date +%Y%m%d_%H%M%S).csv
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from panda_control.config import load_config
from panda_control.remote_client import RemotePandaClient


def _phases_for_mode(mode: str):
    """Return the two ACTIVE phases for the given mode (no settle)."""
    if mode == "x_then_y":
        return [
            ("phase1_x", 2.0, [0.015, 0.0,   0.0]),
            ("phase2_y", 2.0, [0.0,   0.015, 0.0]),
        ]
    if mode == "x_then_x":
        # Same direction in both phases.  Splits a 4-s pure-+x run into two
        # 2-s "phases" with a phase boundary in the middle but NO change in
        # commanded direction.  If x_then_y fails but x_then_x works, the
        # bug is direction change at the boundary.  If x_then_x ALSO fails,
        # the bug is in multi-phase scheduling itself (probably NUC binaries).
        return [
            ("phase1_x", 2.0, [0.015, 0.0, 0.0]),
            ("phase2_x", 2.0, [0.015, 0.0, 0.0]),
        ]
    raise ValueError(f"unknown --mode {mode!r}")


def _with_settle(active_phases, settle_s: float):
    """Interleave a zero-delta ``settle`` hold phase between active phases.

    A settle phase commands ``target_pos = current_pos`` every tick, so the
    impedance controller holds in place and the EE velocity damps to ~0 (time
    constant ~ m/kd ~ 25-70 ms) before the next active phase flips the
    commanded direction.  This is the fix six_dof_pose_test.py already uses.
    ``settle_s <= 0`` returns the active phases unchanged (failure repro).
    """
    if settle_s <= 0.0:
        return list(active_phases)
    out = []
    for i, ph in enumerate(active_phases):
        out.append(ph)
        if i < len(active_phases) - 1:
            out.append((f"settle{i + 1}", float(settle_s), [0.0, 0.0, 0.0]))
    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--mode", choices=["x_then_y", "x_then_x"], default="x_then_y",
                   help="Phase schedule: x_then_y (default, direction-change boundary) or "
                   "x_then_x (no-change boundary; diagnostic for isolating direction-change as trigger).")
    p.add_argument("--config", type=str, default=None, help="Path to robot.yaml.")
    p.add_argument("--rate", type=float, default=50.0,
                   help="Target re-anchor rate [Hz] (default 50).")
    p.add_argument("--settle", type=float, default=0.3,
                   help="Zero-delta hold [s] inserted between the two active phases so "
                   "velocity damps before the direction flips (default 0.3). Pass 0 to "
                   "reproduce the no-settle failure (phase-boundary reflex).")
    p.add_argument("--kp-pos", type=float, default=500.0,
                   help="Cartesian position stiffness (default 500).")
    p.add_argument("--kp-ori", type=float, default=30.0,
                   help="Cartesian orientation stiffness (default 30).")
    p.add_argument("--log", type=str, default=None,
                   help="CSV trajectory log path.  Sidecar <run>.json is written alongside.")
    p.add_argument("--no-return", action="store_true",
                   help="Do NOT command a return to the start pose at the end.")
    p.add_argument("--skip-reset", action="store_true",
                   help="Skip the startup osc_shm cycle (use only if you ran reset_home.py just before).")
    p.add_argument("--reset-speed", type=float, default=None,
                   help="speed_factor for the startup move_to_q (None = config default).")
    return p.parse_args()


def _rot_dev_rad(q_cur_wxyz: np.ndarray, q_init_wxyz: np.ndarray) -> float:
    dot = float(np.clip(abs(np.dot(q_cur_wxyz, q_init_wxyz)), 0.0, 1.0))
    return 2.0 * math.acos(dot)


def main() -> int:
    args = parse_args()

    kp_pos = float(args.kp_pos)
    kp_ori = float(args.kp_ori)
    dt = 1.0 / float(args.rate)
    active_phases = _phases_for_mode(args.mode)
    phases = _with_settle(active_phases, float(args.settle))
    total_duration = sum(p[1] for p in phases)
    # Size buffers from the exact per-phase tick sum (matches the loop below),
    # not round(total*rate), so global_tick can never overrun the buffers.
    n_total = sum(int(round(dur * float(args.rate))) for (_, dur, _) in phases)

    log_path = Path(args.log).expanduser().resolve() if args.log else None
    if log_path is None:
        print("[smoke] (no --log given, trajectory will not be saved)", file=sys.stderr)

    settle_desc = f"{args.settle:.2f}s" if args.settle > 0.0 else "OFF (failure repro)"
    print(
        f"[smoke] mode={args.mode!r}, {len(phases)} phases, total {total_duration:.1f} s @ "
        f"{args.rate:.1f} Hz ({n_total} ticks), settle={settle_desc}"
    )
    for name, dur, pos_d in phases:
        print(f"        {name:10s}  dur={dur:.2f}s  δ_pos={pos_d}")

    cfg = load_config(args.config)
    with RemotePandaClient(cfg) as robot:
        # In-place osc_shm cycle (same logic as fixed_delta_pose_test.py).
        if not args.skip_reset:
            try:
                pre_state = robot.wait_for_state(timeout_s=2.0)
                q_for_reset = np.asarray(pre_state.q, dtype=np.float64)
                reset_kind = "current_q (in-place osc_shm cycle)"
            except TimeoutError:
                q_for_reset = np.asarray(cfg.robot.init_q, dtype=np.float64).reshape(-1)
                reset_kind = "cfg.robot.init_q (recovery)"
            print(f"[smoke] reset_first: {reset_kind}")
            robot.move_to_q(q_for_reset, speed_factor=args.reset_speed)
            print("[smoke] reset_first done; sleeping 1.5 s for SUB warm-up.")
            time.sleep(1.5)

        robot.set_gains(kp_pos=kp_pos, kp_ori=kp_ori)

        state = robot.wait_for_state(timeout_s=10.0)
        init_pos = state.ee_pos.copy()
        anchor_quat_wxyz = state.ee_quat.copy()
        anchor_quat_wxyz /= max(float(np.linalg.norm(anchor_quat_wxyz)), 1e-12)
        print(f"[smoke] start ee_pos = {init_pos}")
        print(f"[smoke] anchor quat (wxyz, FROZEN) = {anchor_quat_wxyz}")

        # Logging buffers (one row per tick + initial row).
        log_t = np.zeros(n_total + 1, dtype=np.float64)
        log_phase_name = [""] * (n_total + 1)
        log_pos = np.zeros((n_total + 1, 3), dtype=np.float64)
        log_quat = np.zeros((n_total + 1, 4), dtype=np.float64)
        log_rotdev = np.zeros(n_total + 1, dtype=np.float64)
        log_speed = np.zeros(n_total + 1, dtype=np.float64)
        log_seq = np.zeros(n_total + 1, dtype=np.int64)
        log_q = np.full((n_total + 1, 7), np.nan, dtype=np.float64)
        log_dq = np.full((n_total + 1, 7), np.nan, dtype=np.float64)

        log_t[0] = 0.0
        log_phase_name[0] = "reset"
        log_pos[0] = init_pos
        log_quat[0] = anchor_quat_wxyz
        log_rotdev[0] = 0.0
        log_speed[0] = 0.0
        log_seq[0] = int(state.seq)
        log_q[0] = np.asarray(state.q, dtype=np.float64)
        log_dq[0] = np.asarray(state.dq, dtype=np.float64)

        # State-stream staleness watchdog.  osc_shm publishes at 1 kHz; at a
        # 50 Hz read loop every get_state() should show a fresh seq.  If seq
        # stops advancing the controller (or the daemon PUB) has died -- the
        # chase would otherwise keep computing targets off a frozen pose for
        # the rest of the run and hide WHEN it died.  Break loudly instead.
        stale_limit = max(5, int(round(0.4 * float(args.rate))))  # ~0.4 s
        last_seq = int(state.seq)
        last_fresh_tick = 0
        last_fresh_t = 0.0
        stale_count = 0
        froze = False

        prev_pos = init_pos.copy()
        prev_t = 0.0
        t0 = time.monotonic()
        global_tick = 0
        try:
            for phase_idx, (phase_name, dur, pos_d) in enumerate(phases, start=1):
                if froze:
                    break
                n_phase = int(round(dur * float(args.rate)))
                delta = np.asarray(pos_d, dtype=np.float64)
                print(f"[smoke] >>> phase {phase_idx} {phase_name} ({n_phase} ticks)")

                for _ in range(n_phase):
                    global_tick += 1
                    fresh = robot.get_state()
                    cur_pos = (
                        np.asarray(fresh.ee_pos, dtype=np.float64)
                        if fresh is not None else prev_pos
                    )
                    cur_quat = (
                        np.asarray(fresh.ee_quat, dtype=np.float64)
                        if fresh is not None else anchor_quat_wxyz
                    )
                    cur_seq = int(fresh.seq) if fresh is not None else last_seq

                    # Pure position chase, orientation locked at anchor.
                    # (Verbatim semantics of fixed_delta_pose_test.py.)
                    target_pos = cur_pos + delta
                    robot.set_ee_target(target_pos, anchor_quat_wxyz)

                    now = time.monotonic() - t0
                    log_t[global_tick] = now
                    log_phase_name[global_tick] = phase_name
                    log_pos[global_tick] = cur_pos
                    log_quat[global_tick] = cur_quat
                    log_rotdev[global_tick] = _rot_dev_rad(cur_quat, anchor_quat_wxyz)
                    seg_dt = max(now - prev_t, 1e-6)
                    log_speed[global_tick] = float(np.linalg.norm(cur_pos - prev_pos) / seg_dt)
                    log_seq[global_tick] = cur_seq
                    if fresh is not None:
                        log_q[global_tick] = np.asarray(fresh.q, dtype=np.float64)
                        log_dq[global_tick] = np.asarray(fresh.dq, dtype=np.float64)
                    prev_pos = cur_pos.copy()
                    prev_t = now

                    # Watchdog: has the state stream advanced?
                    if cur_seq != last_seq:
                        last_seq = cur_seq
                        last_fresh_tick = global_tick
                        last_fresh_t = now
                        stale_count = 0
                    else:
                        stale_count += 1
                        if stale_count >= stale_limit:
                            froze = True
                            print(
                                f"[smoke] !! STATE STREAM FROZE: seq stuck at {last_seq} "
                                f"for {stale_count} ticks. Last fresh frame at tick "
                                f"{last_fresh_tick} (t={last_fresh_t:.3f}s, phase "
                                f"'{log_phase_name[last_fresh_tick]}'). osc_shm likely "
                                f"died here (reflex). Aborting chase.",
                                file=sys.stderr,
                            )
                            break

                    sleep_for = (t0 + global_tick * dt) - time.monotonic()
                    if sleep_for > 0:
                        time.sleep(sleep_for)
        except KeyboardInterrupt:
            print("[smoke] interrupted by user", file=sys.stderr)

        # Truncate buffers to actual ticks completed.
        n_actual = global_tick + 1
        log_t = log_t[:n_actual]
        log_phase_name = log_phase_name[:n_actual]
        log_pos = log_pos[:n_actual]
        log_quat = log_quat[:n_actual]
        log_rotdev = log_rotdev[:n_actual]
        log_speed = log_speed[:n_actual]
        log_seq = log_seq[:n_actual]
        log_q = log_q[:n_actual]
        log_dq = log_dq[:n_actual]

        elapsed = time.monotonic() - t0
        final = robot.get_state()
        final_pos = (
            np.asarray(final.ee_pos, dtype=np.float64) if final is not None else prev_pos
        )
        disp = final_pos - init_pos
        print(
            f"[smoke] net EE displacement = {disp} m "
            f"(|disp| = {np.linalg.norm(disp):.4f} m over {elapsed:.2f} s)."
        )
        print(f"[smoke] achieved loop rate ≈ {global_tick / max(elapsed, 1e-6):.1f} Hz.")

        # Per-phase split (for quick eyeball check).  Reference the ACTIVE
        # phase names so an interleaved settle phase doesn't shift the indices.
        phase1_name = active_phases[0][0]
        phase2_name = active_phases[1][0]
        phase1_rows = [i for i in range(n_actual) if log_phase_name[i] == phase1_name]
        phase2_rows = [i for i in range(n_actual) if log_phase_name[i] == phase2_name]
        if phase1_rows:
            p1_end = log_pos[phase1_rows[-1]] - init_pos
            print(f"[smoke] end-of-{phase1_name} displacement: {p1_end}")
        if phase2_rows:
            # Tick just before phase 2's first tick = end of settle (or end of
            # phase 1 when --settle 0).  Incremental = phase 2's own net motion.
            p2_start = log_pos[phase2_rows[0] - 1] - init_pos
            p2_end = log_pos[phase2_rows[-1]] - init_pos
            print(f"[smoke] {phase2_name} incremental: {p2_end - p2_start}")

        if froze:
            print(
                f"[smoke] !! run aborted by watchdog: controller froze near tick "
                f"{last_fresh_tick} (t={last_fresh_t:.3f}s, phase "
                f"'{log_phase_name[last_fresh_tick]}'). See seq column in the CSV.",
                file=sys.stderr,
            )

        # Write the log FIRST -- the return-to-home below can fail (e.g. the arm
        # latched into REFLEX mid-run), and we must not lose the trajectory that
        # tells us WHERE it died.
        if log_path is not None:
            _write_csv(log_path, log_t, log_phase_name, log_pos, init_pos,
                       log_quat, log_rotdev, log_speed, log_seq, log_q, log_dq)
            _write_meta_json(
                log_path,
                kp_pos=kp_pos, kp_ori=kp_ori,
                mode=args.mode,
                settle_s=float(args.settle),
                phases=phases,
                duration_s=float(total_duration),
                refresh_rate_hz_target=float(args.rate),
                refresh_rate_hz_achieved=float(global_tick / max(elapsed, 1e-6)),
                n_ticks=int(global_tick),
                start_ee_pos=init_pos,
                start_ee_quat_wxyz=anchor_quat_wxyz,
                final_disp=disp,
                froze=bool(froze),
                froze_at_tick=int(last_fresh_tick) if froze else None,
                froze_at_t_s=float(last_fresh_t) if froze else None,
            )

        # Safe return to home (via move_to_q -- raw set_ee_target jump could trip
        # clamp).  Non-fatal: a failure here must not discard the run.
        if not args.no_return:
            q_home = np.asarray(cfg.robot.init_q, dtype=np.float64).reshape(-1)
            print(f"[smoke] returning to home via move_to_q(init_q) ...")
            try:
                robot.move_to_q(q_home, speed_factor=args.reset_speed)
                print("[smoke] return done.")
            except Exception as e:
                print(
                    f"[smoke] !! return-to-home FAILED: {e}\n"
                    f"[smoke]    (arm is likely latched in REFLEX; clear it via "
                    f"Franka Desk before the next run.)",
                    file=sys.stderr,
                )
    return 0


def _write_csv(path, t, phase_name, pos, init_pos, quat_wxyz, rotdev, speed, seq, q, dq):
    disp = pos - init_pos[None, :]
    columns = ["t_s", "phase_name", "x", "y", "z",
               "disp_x", "disp_y", "disp_z",
               "qw", "qx", "qy", "qz",
               "rot_dev_rad", "speed_mps", "seq"]
    columns += [f"q{j}" for j in range(7)] + [f"dq{j}" for j in range(7)]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write(",".join(columns) + "\n")
        for i in range(len(t)):
            q_str = ",".join(f"{q[i,j]:.9f}" for j in range(7))
            dq_str = ",".join(f"{dq[i,j]:.9f}" for j in range(7))
            f.write(
                f"{t[i]:.9f},{phase_name[i]},"
                f"{pos[i,0]:.9f},{pos[i,1]:.9f},{pos[i,2]:.9f},"
                f"{disp[i,0]:.9f},{disp[i,1]:.9f},{disp[i,2]:.9f},"
                f"{quat_wxyz[i,0]:.9f},{quat_wxyz[i,1]:.9f},"
                f"{quat_wxyz[i,2]:.9f},{quat_wxyz[i,3]:.9f},"
                f"{rotdev[i]:.9f},{speed[i]:.9f},{int(seq[i])},"
                f"{q_str},{dq_str}\n"
            )
    print(f"[smoke] wrote CSV: {path}")


def _write_meta_json(csv_path, **meta):
    json_path = csv_path.with_suffix(".json")
    payload = {
        "control": {"kp_pos": float(meta["kp_pos"]), "kp_ori": float(meta["kp_ori"])},
        "command": {
            "mode": meta.get("mode", "unknown"),
            "settle_s": float(meta.get("settle_s", 0.0)),
            "phases": [{"name": n, "duration_s": d, "delta_pos_m": list(p)}
                       for (n, d, p) in meta["phases"]],
            "duration_s_total": float(meta["duration_s"]),
            "refresh_rate_hz_target": float(meta["refresh_rate_hz_target"]),
            "refresh_rate_hz_achieved": float(meta["refresh_rate_hz_achieved"]),
            "n_ticks": int(meta["n_ticks"]),
        },
        "start_state": {
            "ee_pos_m": [float(v) for v in meta["start_ee_pos"]],
            "ee_quat_wxyz": [float(v) for v in meta["start_ee_quat_wxyz"]],
        },
        "result": {
            "final_disp_m": [float(v) for v in meta["final_disp"]],
            "froze": bool(meta.get("froze", False)),
            "froze_at_tick": meta.get("froze_at_tick"),
            "froze_at_t_s": meta.get("froze_at_t_s"),
        },
        "source": {"script": "panda_control/examples/two_phase_smoke_test.py"},
    }
    json_path.write_text(json.dumps(payload, indent=2))
    print(f"[smoke] wrote meta: {json_path}")


if __name__ == "__main__":
    raise SystemExit(main())
