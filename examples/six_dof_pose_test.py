"""Real-arm replica of the ``Franka-Debug-v2`` 7-phase 6-DOF stress test.

Drives the real Franka through a sequential set of single-axis-then-combined
motion phases using the SAME control law and per-tick deltas as the IsaacLab
``franka_debug_v2_six_dof_test.py`` script:

    Phase | t (s)  | δ_pos (m/tick)               | δ_rot (rad/tick)
    1     | 0-2    | (+0.015, 0,     0)           | (0,     0,     0)
    2     | 2-4    | (0,     +0.015, 0)           | (0,     0,     0)
    3     | 4-6    | (0,      0,    +0.015)       | (0,     0,     0)
    4     | 6-8    | (0,      0,     0)           | (+0.05, 0,     0)
    5     | 8-10   | (0,      0,     0)           | (0,    +0.05,  0)
    6     | 10-12  | (0,      0,     0)           | (0,     0,    +0.05)
    7     | 12-14  | (+0.008,+0.008,+0.008)       | (+0.025,+0.025,+0.025)

Per tick (matches v2 sim):
    target_pos  = current_pos  + δ_pos
    target_quat = exp_world(δ_rot)  ⊗  current_quat       (world-frame rotate)

When δ_rot = 0 → target_quat = current_quat (orientation released; PD applies
no orientation force).  When δ_pos = 0 → target_pos = current_pos (position
released).  This is symmetric to v2's position-only fixed-delta test.

Defaults match the sim run we're trying to compare against:
    kp_pos=500, kp_ori=30 (UWTaskImpCtrlCfg / Franka-Debug-v2 parity).

Self-contained reset: calls ``move_to_q(current_q)`` at startup to cycle
osc_shm without moving the arm; if the SUB state stream is dead falls back to
``move_to_q(cfg.robot.init_q)`` as a recovery move.  Then ``move_to_q`` again
at the end to return the arm cleanly to home (since the 7-phase test ends at
a non-trivial pose; a raw set_ee_target return could trigger the per-tick
clamp).

Writes <run>.csv + <run>.json metadata sidecar so the downstream sim2real
diff tool can identify the run setup without filename parsing.

Examples::

    # Run with sim-matched defaults; let the script reset automatically.
    python examples/six_dof_pose_test.py \\
        --log data/real_sixdof_$(date +%Y%m%d_%H%M%S).csv

    # Override gains / rate.
    python examples/six_dof_pose_test.py --kp-pos 500 --kp-ori 30 --rate 50 \\
        --log data/real_sixdof_kp500_$(date +%Y%m%d_%H%M%S).csv
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

# ---------------------------------------------------------------------------
# Phase schedule (verbatim copy of the sim script's PHASES).
# (name, duration_s, [δ_x, δ_y, δ_z] m/tick, [δ_rx, δ_ry, δ_rz] rad/tick)
# ---------------------------------------------------------------------------
# Settle periods between active phases are CRITICAL on real: without them the
# sudden direction change at the phase boundary (arm has ~5 cm/s in phase N's
# direction, the next tick suddenly commands phase N+1's direction) drives
# joint jerk past libfranka's safety thresholds and the arm latches into the
# REFLEX mode, killing osc_shm and silently freezing the rest of the test.
# 0.3 s is far more than the Kp/Kd time constant (~25 ms) so velocity damps
# cleanly to 0 before the next command.
_S = 0.3
PHASES = [
    ("phase1_x",    2.0, [0.015, 0.0,   0.0  ], [0.0,   0.0,   0.0  ]),
    ("settle1",     _S,  [0.0,   0.0,   0.0  ], [0.0,   0.0,   0.0  ]),
    ("phase2_y",    2.0, [0.0,   0.015, 0.0  ], [0.0,   0.0,   0.0  ]),
    ("settle2",     _S,  [0.0,   0.0,   0.0  ], [0.0,   0.0,   0.0  ]),
    ("phase3_z",    2.0, [0.0,   0.0,   0.015], [0.0,   0.0,   0.0  ]),
    ("settle3",     _S,  [0.0,   0.0,   0.0  ], [0.0,   0.0,   0.0  ]),
    ("phase4_rx",   2.0, [0.0,   0.0,   0.0  ], [0.05,  0.0,   0.0  ]),
    ("settle4",     _S,  [0.0,   0.0,   0.0  ], [0.0,   0.0,   0.0  ]),
    ("phase5_ry",   2.0, [0.0,   0.0,   0.0  ], [0.0,   0.05,  0.0  ]),
    ("settle5",     _S,  [0.0,   0.0,   0.0  ], [0.0,   0.0,   0.0  ]),
    ("phase6_rz",   2.0, [0.0,   0.0,   0.0  ], [0.0,   0.0,   0.05 ]),
    ("settle6",     _S,  [0.0,   0.0,   0.0  ], [0.0,   0.0,   0.0  ]),
    ("phase7_all6", 2.0, [0.008, 0.008, 0.008], [0.025, 0.025, 0.025]),
]

# ---- Defaults matched to the sim Franka-Debug-v2 run ----------------------
V2_KP_POS = 500.0
V2_KP_ORI = 30.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--config", type=str, default=None, help="Path to robot.yaml.")
    p.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Target re-anchor rate [Hz] (default 50). Sim uses 1000 Hz physics; "
        "rate-mismatch caveat applies (see fixed_delta_pose_test.py docstring).",
    )
    p.add_argument(
        "--kp-pos",
        type=float,
        default=V2_KP_POS,
        help=f"Cartesian position stiffness (default {V2_KP_POS:.0f}, matches v2 sim).",
    )
    p.add_argument(
        "--kp-ori",
        type=float,
        default=V2_KP_ORI,
        help=f"Cartesian orientation stiffness (default {V2_KP_ORI:.0f}, matches v2 sim).",
    )
    p.add_argument(
        "--log",
        type=str,
        default=None,
        help="CSV trajectory log path.  Sidecar <run>.json is written alongside.",
    )
    p.add_argument(
        "--no-return",
        action="store_true",
        help="Do NOT move the arm back to home pose at the end of the run.",
    )
    p.add_argument(
        "--skip-reset",
        action="store_true",
        help=(
            "Skip the startup osc_shm cycle.  By default the script calls "
            "move_to_q(q_current) so the daemon stops -> restarts osc_shm "
            "(necessary if a prior abort left osc_shm dead).  Fall-back: if "
            "SUB state stream is dead at startup, falls back to "
            "move_to_q(cfg.robot.init_q) which physically moves to home."
        ),
    )
    p.add_argument(
        "--reset-speed",
        type=float,
        default=None,
        help="speed_factor for the startup / return move_to_q calls (0,0.5]; None = config default.",
    )
    return p.parse_args()


# ----------------------------------------------------------- helpers
def _quat_mul_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=np.float64)


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def _axis_angle_to_quat_wxyz(vec: np.ndarray) -> np.ndarray:
    """3-vec (axis * angle in rad) -> wxyz unit quaternion.  Identity if |vec|≈0."""
    angle = float(np.linalg.norm(vec))
    if angle < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    half = 0.5 * angle
    axis = vec / angle
    return np.array([math.cos(half), *(math.sin(half) * axis)], dtype=np.float64)


def _rot_dev_rad(q_cur_wxyz: np.ndarray, q_init_wxyz: np.ndarray) -> float:
    """Shortest-path geodesic angle [rad] between two wxyz quaternions."""
    dot = float(np.clip(abs(np.dot(q_cur_wxyz, q_init_wxyz)), 0.0, 1.0))
    return 2.0 * math.acos(dot)


def _robust_wait_for_state(robot, timeout_s: float = 10.0, poll_dt: float = 0.1):
    """Wait for a fresh state frame, falling back to synchronous REQ get_state
    (matches the helper in fixed_delta_pose_test.py)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        s = robot.get_state()
        if s is not None:
            return s
        s = robot.get_state(fresh=True)
        if s is not None:
            return s
        time.sleep(poll_dt)
    raise TimeoutError(f"no state frame received via SUB cache or REQ in {timeout_s:.1f}s")


# ----------------------------------------------------------- main
def main() -> int:
    args = parse_args()

    kp_pos = float(args.kp_pos)
    kp_ori = float(args.kp_ori)
    kd_pos = 2.0 * math.sqrt(kp_pos)
    kd_ori = 2.0 * math.sqrt(kp_ori)

    dt = 1.0 / float(args.rate)
    total_duration = sum(p[1] for p in PHASES)
    n_total = int(round(total_duration * float(args.rate)))

    log_path = Path(args.log).expanduser().resolve() if args.log else None
    if log_path is None:
        print("[six_dof] (no --log given, trajectory will not be saved)", file=sys.stderr)

    print(
        f"[six_dof] {len(PHASES)} phases, total {total_duration:.1f} s @ "
        f"{args.rate:.1f} Hz ({n_total} ticks)"
    )
    print(
        f"[six_dof] gains: kp_pos={kp_pos:.1f}, kp_ori={kp_ori:.1f}  "
        f"(daemon kd ≈ kd_pos={kd_pos:.2f}, kd_ori={kd_ori:.2f})."
    )
    for name, dur, pos_d, rot_d in PHASES:
        print(f"          {name:12s}  dur={dur:.2f}s  δ_pos={pos_d}  δ_rot={rot_d}")

    cfg = load_config(args.config)
    with RemotePandaClient(cfg) as robot:
        # Self-contained reset (cycle osc_shm in-place, or recover to home if SUB dead).
        if not args.skip_reset:
            try:
                pre_state = robot.wait_for_state(timeout_s=2.0)
                q_for_reset = np.asarray(pre_state.q, dtype=np.float64)
                reset_kind = "current_q (in-place osc_shm cycle)"
            except TimeoutError:
                q_for_reset = np.asarray(cfg.robot.init_q, dtype=np.float64).reshape(-1)
                reset_kind = (
                    "cfg.robot.init_q (state stream dead -- recovery move "
                    "physically goes home)"
                )
            print(f"[six_dof] reset_first: target = {reset_kind}")
            robot.move_to_q(q_for_reset, speed_factor=args.reset_speed)
            print("[six_dof] reset_first done; osc_shm cycled.")
            time.sleep(2.0)  # SUB warm-up

        # Set gains to match sim Kp.
        robot.set_gains(kp_pos=kp_pos, kp_ori=kp_ori)

        state = _robust_wait_for_state(robot, timeout_s=10.0)
        sub_cache_alive = robot.get_state() is not None
        print(
            f"[six_dof] state stream healthy "
            f"(SUB cache {'alive' if sub_cache_alive else 'empty - using REQ fallback'})."
        )

        init_pos = np.asarray(state.ee_pos, dtype=np.float64).copy()
        init_quat = _normalize_quat(np.asarray(state.ee_quat, dtype=np.float64))
        print(f"[six_dof] start ee_pos  = {init_pos}")
        print(f"[six_dof] start ee_quat (wxyz) = {init_quat}")

        # Control loop ALWAYS uses synchronous REQ (`fresh=True`) instead of
        # the SUB cache.  Rationale (the bug we hit before): the chase
        # semantics (target = cur + δ) need a fresh cur each tick.  If the
        # SUB cache ever stalls mid-run -- ZMQ slow-joiner side effect, brief
        # daemon PUB pause, REQ contention starving the SUB poller -- the
        # cached `cur_pos` freezes at its last value, every subsequent
        # `set_ee_target` ships the same stale-pos+δ target, the arm reaches
        # it once and then sits there for the rest of the test.  REQ
        # round-trips ~1 ms at 50 Hz so the 5% overhead is fine and we get a
        # guaranteed-fresh frame every tick.
        get_state_fn = (lambda: robot.get_state(fresh=True))
        print("[six_dof] control loop using synchronous REQ get_state (fresh=True).")

        # Pre-allocate logging buffers.
        log_t = np.zeros(n_total + 1, dtype=np.float64)
        log_phase_idx = np.zeros(n_total + 1, dtype=np.int32)
        log_phase_name = [""] * (n_total + 1)
        log_pos = np.zeros((n_total + 1, 3), dtype=np.float64)
        log_quat = np.zeros((n_total + 1, 4), dtype=np.float64)
        log_rotdev = np.zeros(n_total + 1, dtype=np.float64)
        log_speed = np.zeros(n_total + 1, dtype=np.float64)

        # Row 0: t=0, init pose.
        log_t[0] = 0.0
        log_phase_idx[0] = 0
        log_phase_name[0] = "reset"
        log_pos[0] = init_pos
        log_quat[0] = init_quat
        log_rotdev[0] = 0.0
        log_speed[0] = 0.0

        prev_pos = init_pos.copy()
        prev_t = 0.0
        t0 = time.monotonic()
        global_tick = 0
        try:
            for phase_idx, (phase_name, dur, pos_d, rot_d) in enumerate(PHASES, start=1):
                n_phase = int(round(dur * float(args.rate)))
                pos_d_arr = np.asarray(pos_d, dtype=np.float64)
                rot_d_arr = np.asarray(rot_d, dtype=np.float64)
                print(f"[six_dof] >>> phase {phase_idx} {phase_name} ({n_phase} ticks)")

                for _ in range(n_phase):
                    global_tick += 1
                    fresh = get_state_fn()
                    if fresh is not None:
                        cur_pos = np.asarray(fresh.ee_pos, dtype=np.float64)
                        cur_quat = _normalize_quat(np.asarray(fresh.ee_quat, dtype=np.float64))
                    else:
                        cur_pos = prev_pos
                        cur_quat = init_quat

                    # Pure chase semantics for both position and orientation
                    # (symmetric, matches sim).  Validity depends on
                    # `cur_pos`/`cur_quat` being FRESH each tick -- see the
                    # get_state_fn definition below for why we always force
                    # synchronous REQ rather than reading SUB cache.
                    target_pos = cur_pos + pos_d_arr
                    delta_q = _axis_angle_to_quat_wxyz(rot_d_arr)
                    target_quat = _normalize_quat(_quat_mul_wxyz(delta_q, cur_quat))
                    robot.set_ee_target(target_pos, target_quat)

                    # Log.
                    now = time.monotonic() - t0
                    log_t[global_tick] = now
                    log_phase_idx[global_tick] = phase_idx
                    log_phase_name[global_tick] = phase_name
                    log_pos[global_tick] = cur_pos
                    log_quat[global_tick] = cur_quat
                    log_rotdev[global_tick] = _rot_dev_rad(cur_quat, init_quat)
                    seg_dt = max(now - prev_t, 1e-6)
                    log_speed[global_tick] = float(np.linalg.norm(cur_pos - prev_pos) / seg_dt)
                    prev_pos = cur_pos.copy()
                    prev_t = now

                    sleep_for = (t0 + global_tick * dt) - time.monotonic()
                    if sleep_for > 0:
                        time.sleep(sleep_for)
        except KeyboardInterrupt:
            print("[six_dof] interrupted by user", file=sys.stderr)

        # Truncate buffers to actual ticks completed.
        n_actual = global_tick + 1  # row 0 + ticks
        log_t = log_t[:n_actual]
        log_phase_idx = log_phase_idx[:n_actual]
        log_phase_name = log_phase_name[:n_actual]
        log_pos = log_pos[:n_actual]
        log_quat = log_quat[:n_actual]
        log_rotdev = log_rotdev[:n_actual]
        log_speed = log_speed[:n_actual]

        elapsed = time.monotonic() - t0
        final = robot.get_state() if sub_cache_alive else robot.get_state(fresh=True)
        final_pos = (
            np.asarray(final.ee_pos, dtype=np.float64) if final is not None else prev_pos
        )
        disp = final_pos - init_pos
        print(
            f"[six_dof] net EE displacement = {disp} m "
            f"(|disp| = {np.linalg.norm(disp):.4f} m over {elapsed:.2f} s)."
        )
        print(f"[six_dof] final EE rot deviation from start = {log_rotdev[-1]:.4f} rad "
              f"({log_rotdev[-1] * 57.2958:.2f}°).")
        print(f"[six_dof] achieved loop rate ≈ {global_tick / max(elapsed, 1e-6):.1f} Hz.")

        # Safe return to home via move_to_q (raw set_ee_target jump could trip the
        # daemon per-tick clamp because the 7-phase test ends at a non-trivial pose).
        if not args.no_return:
            q_home = np.asarray(cfg.robot.init_q, dtype=np.float64).reshape(-1)
            print(f"[six_dof] returning to home via move_to_q(init_q) ...")
            robot.move_to_q(q_home, speed_factor=args.reset_speed)
            print("[six_dof] return done.")

        if log_path is not None:
            _write_csv(
                log_path, log_t, log_phase_idx, log_phase_name,
                log_pos, init_pos, log_quat, log_rotdev, log_speed,
            )
            _write_meta_json(
                log_path,
                kp_pos=kp_pos,
                kp_ori=kp_ori,
                kd_pos_critical=kd_pos,
                kd_ori_critical=kd_ori,
                phases=PHASES,
                duration_s=float(total_duration),
                refresh_rate_hz_target=float(args.rate),
                refresh_rate_hz_achieved=float(global_tick / max(elapsed, 1e-6)),
                n_ticks=int(global_tick),
                start_ee_pos=init_pos,
                start_ee_quat_wxyz=init_quat,
                final_disp=disp,
                final_rot_dev_rad=float(log_rotdev[-1]),
            )
    return 0


def _write_csv(
    path: Path,
    t: np.ndarray,
    phase_idx: np.ndarray,
    phase_name: list[str],
    pos: np.ndarray,
    init_pos: np.ndarray,
    quat_wxyz: np.ndarray,
    rotdev: np.ndarray,
    speed: np.ndarray,
) -> None:
    disp = pos - init_pos[None, :]
    columns = [
        "t_s", "phase_idx", "phase_name",
        "x", "y", "z",
        "disp_x", "disp_y", "disp_z",
        "qw", "qx", "qy", "qz",
        "rot_dev_rad", "speed_mps",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write(",".join(columns) + "\n")
        for i in range(len(t)):
            f.write(
                f"{t[i]:.9f},{phase_idx[i]},{phase_name[i]},"
                f"{pos[i, 0]:.9f},{pos[i, 1]:.9f},{pos[i, 2]:.9f},"
                f"{disp[i, 0]:.9f},{disp[i, 1]:.9f},{disp[i, 2]:.9f},"
                f"{quat_wxyz[i, 0]:.9f},{quat_wxyz[i, 1]:.9f},"
                f"{quat_wxyz[i, 2]:.9f},{quat_wxyz[i, 3]:.9f},"
                f"{rotdev[i]:.9f},{speed[i]:.9f}\n"
            )
    print(f"[six_dof] wrote CSV: {path}")


def _write_meta_json(
    csv_path: Path,
    *,
    kp_pos: float,
    kp_ori: float,
    kd_pos_critical: float,
    kd_ori_critical: float,
    phases: list,
    duration_s: float,
    refresh_rate_hz_target: float,
    refresh_rate_hz_achieved: float,
    n_ticks: int,
    start_ee_pos: np.ndarray,
    start_ee_quat_wxyz: np.ndarray,
    final_disp: np.ndarray,
    final_rot_dev_rad: float,
) -> None:
    json_path = csv_path.with_suffix(".json")
    meta = {
        "control": {
            "kp_pos": float(kp_pos),
            "kp_ori": float(kp_ori),
            "kd_pos_expected_critical": float(kd_pos_critical),
            "kd_ori_expected_critical": float(kd_ori_critical),
            "note": "kd is derived by the daemon as 2*sqrt(kp); values here are the analytic critical-damping reference only.",
        },
        "command": {
            "phases": [
                {
                    "name": name,
                    "duration_s": float(dur),
                    "delta_pos_m": [float(v) for v in pd],
                    "delta_rot_rad": [float(v) for v in rd],
                }
                for (name, dur, pd, rd) in phases
            ],
            "duration_s_total": float(duration_s),
            "refresh_rate_hz_target": float(refresh_rate_hz_target),
            "refresh_rate_hz_achieved": float(refresh_rate_hz_achieved),
            "n_ticks": int(n_ticks),
        },
        "start_state": {
            "ee_pos_m": [float(v) for v in start_ee_pos],
            "ee_quat_wxyz": [float(v) for v in start_ee_quat_wxyz],
        },
        "result": {
            "final_disp_m": [float(v) for v in final_disp],
            "final_disp_norm_m": float(np.linalg.norm(final_disp)),
            "final_rot_dev_rad": float(final_rot_dev_rad),
            "final_rot_dev_deg": float(final_rot_dev_rad * 180.0 / math.pi),
        },
        "source": {
            "script": "panda_control/examples/six_dof_pose_test.py",
            "sim_counterpart": "IsaacLab/scripts/environments/franka_debug_v2_six_dof_test.py",
            "csv": csv_path.name,
        },
    }
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(meta, indent=2))
    print(f"[six_dof] wrote meta: {json_path}")


if __name__ == "__main__":
    raise SystemExit(main())
