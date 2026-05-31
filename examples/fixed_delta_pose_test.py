"""Real-arm replica of the ``Franka-Debug-v2`` fixed-delta-action test.

This drives the real Franka with the SAME control scheme as the IsaacLab
``Franka-Debug-v2`` task (see
``IsaacLab/source/isaaclab_tasks/.../direct/Franka_Debug``) and records how the
EE pose evolves, so the real trajectory can be diffed against the sim one
produced by ``scripts/environments/franka_debug_v1_test.py``.

What ``Franka-Debug-v2`` does (inherited from v1, env step = 1 ms):

    ctrl_target_fingertip_midpoint_pos  = fingertip_midpoint_pos + action   # delta
    ctrl_target_fingertip_midpoint_quat = (held at the initial orientation)

i.e. at every control tick the position target is re-anchored to the CURRENT
EE position plus a fixed delta, while the orientation target is frozen at the
pose captured at startup.  With a constant delta this produces a constant task-
space position error of ``|delta|`` and therefore steady motion at roughly::

    v_ss ≈ (kp_pos / kd_pos) * |delta|

Matched-to-v2 defaults (do not need to be passed):

  * delta   = [0.01, 0.0, 0.0] m   (per-tick base-frame delta; v2/v1 default)
  * kp_pos  = 200,  kp_ori = 20    (v2 OSCCtrlCfg.task_prop_gains)
  * kd      = 2*sqrt(kp)           (critical damping; the daemon derives this
                                    identically to IsaacLab get_deriv_gains)
  * duration = 2.0 s               (franka_debug_v1_test.py default)

IMPORTANT behaviour note (sim vs real):
  The sim re-anchors ``target = current + delta`` at 1 kHz (its physics step).
  This Python loop re-anchors at ``--rate`` Hz (default 50).  Because the EE
  partially closes the gap to the target between refreshes, a lower refresh
  rate yields a smaller *mean* tracking error and therefore a slower steady-
  state velocity than the sim.  To approach the sim's behaviour, raise
  ``--rate``.  The achieved rate is reported at the end.

Prerequisites:
  * On the NUC:  python -m panda_control.daemon --config config/robot.yaml
  * On this PC:  pip install -e .   (or PYTHONPATH=python)
  * Arm parked at a sensible start pose (e.g. run examples/reset_home.py first).

Examples:
  # Exact v2 defaults (1 cm/tick in +x for 2 s, kp_pos=200/kp_ori=20):
  python examples/fixed_delta_pose_test.py

  # Same gains, push in +z, log the trajectory for a sim2real diff:
  python examples/fixed_delta_pose_test.py --delta 0.0 0.0 0.01 \\
      --log data/real_delta_z_$(date +%Y%m%d_%H%M%S).csv
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

# ---- Franka-Debug-v2 defaults (mirror the IsaacLab cfg verbatim) -----------
# OSCCtrlCfg.task_prop_gains = [200, 200, 200, 20, 20, 20] -> kp_pos=200, kp_ori=20.
V2_KP_POS = 200.0
V2_KP_ORI = 20.0
# franka_debug_v1_test.py: default --delta [0.01, 0.0, 0.0], --duration_s 2.0.
V2_DELTA_M = (0.01, 0.0, 0.0)
V2_DURATION_S = 2.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--config", type=str, default=None, help="Path to robot.yaml.")
    p.add_argument(
        "--delta",
        type=float,
        nargs=3,
        default=list(V2_DELTA_M),
        metavar=("DX", "DY", "DZ"),
        help="Per-tick base-frame delta added to the EE target [m] "
        "(default: matches Franka-Debug-v2 = 0.01 0.0 0.0).",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=V2_DURATION_S,
        help="How long to apply the delta [s] (default: 2.0, matches v2 test).",
    )
    p.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Target re-anchor rate [Hz] (default: 50). Sim uses 1000 Hz; see "
        "the module docstring on the sim/real rate caveat.",
    )
    p.add_argument(
        "--kp-pos",
        type=float,
        default=V2_KP_POS,
        help="Cartesian position stiffness (default: 200, matches v2).",
    )
    p.add_argument(
        "--kp-ori",
        type=float,
        default=V2_KP_ORI,
        help="Cartesian orientation stiffness (default: 20, matches v2).",
    )
    p.add_argument(
        "--log",
        type=str,
        default=None,
        help="CSV trajectory log path. If unset, nothing is written to disk.",
    )
    p.add_argument(
        "--no-return",
        action="store_true",
        help="Do NOT command a return to the start pose at the end of the run.",
    )
    p.add_argument(
        "--skip-reset",
        action="store_true",
        help=(
            "Skip the startup osc_shm cycle. By default the script calls "
            "move_to_q(q_current) at startup as a no-op-motion trick that "
            "forces the daemon to stop -> restart osc_shm.  Without this "
            "cycle, a prior abort in osc_shm leaves the controller dead "
            "and all subsequent set_ee_target writes are silently ignored. "
            "If the SUB state stream is dead at startup (so we can't read "
            "q_current) we fall back to move_to_q(cfg.robot.init_q), which "
            "DOES physically move the arm home as a recovery path.  Pass "
            "--skip-reset only when you've just run reset_home.py yourself."
        ),
    )
    p.add_argument(
        "--reset-speed",
        type=float,
        default=None,
        help="speed_factor for the startup move_to_q reset (0,0.5]; None = use config default.",
    )
    return p.parse_args()


def _rot_dev_rad(q_cur_wxyz: np.ndarray, q_init_wxyz: np.ndarray) -> float:
    """Shortest-path geodesic angle [rad] between two wxyz quaternions."""
    dot = float(np.clip(abs(np.dot(q_cur_wxyz, q_init_wxyz)), 0.0, 1.0))
    return 2.0 * math.acos(dot)


def _robust_wait_for_state(robot, timeout_s: float = 10.0, poll_dt: float = 0.1):
    """Wait for a fresh state frame, trying the ZMQ SUB cache first and
    falling back to a synchronous ``get_state(fresh=True)`` REQ round-trip.

    The SUB stream can take several seconds to repopulate the client cache
    after ``move_to_q`` cycles osc_shm (root cause unclear; daemon-side PUB
    loop appears fine, so we suspect ZMQ slow-joiner / TCP-buffer effects
    around the osc_shm restart).  REQ is unaffected by this and always
    returns the latest shm frame as soon as osc_shm starts publishing.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        s = robot.get_state()  # SUB cache, fast path
        if s is not None:
            return s
        s = robot.get_state(fresh=True)  # synchronous REQ, fallback
        if s is not None:
            return s
        time.sleep(poll_dt)
    raise TimeoutError(
        f"no state frame received via SUB cache or REQ in {timeout_s:.1f}s"
    )


def main() -> int:
    args = parse_args()

    delta = np.asarray(args.delta, dtype=np.float64)
    kp_pos = float(args.kp_pos)
    kp_ori = float(args.kp_ori)
    # Critical-damping kd the daemon will use (kept only for the printout; we
    # let the daemon derive it so it matches IsaacLab get_deriv_gains exactly).
    kd_pos = 2.0 * math.sqrt(kp_pos)
    kd_ori = 2.0 * math.sqrt(kp_ori)
    v_ss_expected = (kp_pos / kd_pos) * float(np.linalg.norm(delta))

    dt = 1.0 / float(args.rate)
    n = int(round(float(args.duration) * float(args.rate)))
    if n < 1:
        print("[fixed_delta] duration*rate < 1 tick; nothing to do.", file=sys.stderr)
        return 1

    log_path = Path(args.log).expanduser().resolve() if args.log else None
    if log_path is None:
        print("[fixed_delta] (no --log given, trajectory will not be saved)", file=sys.stderr)

    print(
        f"[fixed_delta] delta(per-tick) = {delta.tolist()} m, |delta| = "
        f"{np.linalg.norm(delta):.4f} m; duration = {args.duration:.2f} s @ "
        f"{args.rate:.1f} Hz ({n} ticks)."
    )
    print(
        f"[fixed_delta] gains: kp_pos={kp_pos:.1f}, kp_ori={kp_ori:.1f} "
        f"(daemon kd≈ kd_pos={kd_pos:.2f}, kd_ori={kd_ori:.2f}); "
        f"expected steady-state speed ≈ {v_ss_expected:.4f} m/s."
    )

    cfg = load_config(args.config)
    with RemotePandaClient(cfg) as robot:
        # Self-contained reset: cycle osc_shm WITHOUT moving the arm.
        # `move_to_q` is the only daemon op that stops -> restarts osc_shm
        # (osc_shm.cpp exits on any abort and the daemon does NOT auto-
        # restart it; once dead, set_ee_target writes to shm are no-ops).
        # We call move_to_q(q_current) so the C++ MotionGenerator produces
        # an effective no-op trajectory: osc_shm gets recycled but the arm
        # stays where it was.
        #
        # Fallback path: if the SUB state stream is already dead at startup
        # (osc_shm aborted from a prior run), we can't read q_current.  In
        # that case we use cfg.robot.init_q as the recovery target -- this
        # IS the only branch that physically moves the arm.
        if not args.skip_reset:
            try:
                pre_state = robot.wait_for_state(timeout_s=2.0)
                q_for_reset = np.asarray(pre_state.q, dtype=np.float64)
                reset_kind = "current_q (in-place osc_shm cycle)"
            except TimeoutError:
                q_for_reset = np.asarray(cfg.robot.init_q, dtype=np.float64).reshape(-1)
                reset_kind = (
                    "cfg.robot.init_q (state stream dead -- "
                    "recovery move physically goes home)"
                )

            if q_for_reset.shape != (7,):
                raise ValueError(
                    f"reset target must be 7 floats, got shape {q_for_reset.shape}"
                )
            print(f"[fixed_delta] reset_first: target = {reset_kind}")
            print(
                f"[fixed_delta] reset_first: q={q_for_reset.tolist()} "
                f"speed={args.reset_speed}"
            )
            robot.move_to_q(q_for_reset, speed_factor=args.reset_speed)
            print("[fixed_delta] reset_first done; osc_shm cycled.")

            # After osc_shm restart, the SUB state stream may take a few
            # seconds to repopulate the client cache.  REQ get_state is
            # unaffected (it reads shm directly).  Sleep briefly to give
            # SUB a chance, then fall through to _robust_wait_for_state
            # below which uses REQ as fallback if SUB still hasn't recovered.
            time.sleep(2.0)

        # Match Franka-Debug-v2's task_prop_gains. kd is left unset so the
        # daemon applies its own 2*sqrt(kp) (== IsaacLab get_deriv_gains).
        robot.set_gains(kp_pos=kp_pos, kp_ori=kp_ori)

        state = _robust_wait_for_state(robot, timeout_s=10.0)
        sub_cache_alive = robot.get_state() is not None
        print(
            f"[fixed_delta] state stream healthy "
            f"(SUB cache {'alive' if sub_cache_alive else 'still empty - using REQ fallback'})."
        )
        init_pos = state.ee_pos.copy()
        # Orientation target is frozen at the startup pose (v1: "orientation
        # target left at the initial fingertip orientation").
        anchor_quat_wxyz = state.ee_quat.copy()
        anchor_quat_wxyz /= max(float(np.linalg.norm(anchor_quat_wxyz)), 1e-12)
        print(f"[fixed_delta] start ee_pos = {init_pos}")
        print(f"[fixed_delta] frozen ee_quat (wxyz) = {anchor_quat_wxyz}")

        # Logging buffers.
        log_t = np.zeros(n, dtype=np.float64)
        log_pos = np.zeros((n, 3), dtype=np.float64)
        log_quat = np.zeros((n, 4), dtype=np.float64)
        log_rotdev = np.zeros(n, dtype=np.float64)
        log_speed = np.zeros(n, dtype=np.float64)

        prev_pos = init_pos.copy()
        prev_t = 0.0
        t0 = time.monotonic()
        # If the SUB cache was dead at startup we use REQ fresh=True in the
        # loop; cost is ~ms per tick at 50 Hz which is well within budget.
        get_state_fn = (
            (lambda: robot.get_state())
            if sub_cache_alive
            else (lambda: robot.get_state(fresh=True))
        )
        try:
            for i in range(n):
                fresh = get_state_fn()
                cur_pos = fresh.ee_pos if fresh is not None else prev_pos
                cur_quat = (
                    fresh.ee_quat if fresh is not None else anchor_quat_wxyz
                )
                # Core v1/v2 control law: re-anchor target to CURRENT pos + delta,
                # hold the orientation at the startup pose.
                target_pos = cur_pos + delta
                robot.set_ee_target(target_pos, anchor_quat_wxyz)

                now = time.monotonic() - t0
                log_t[i] = now
                log_pos[i] = cur_pos
                log_quat[i] = cur_quat
                log_rotdev[i] = _rot_dev_rad(cur_quat, anchor_quat_wxyz)
                seg_dt = max(now - prev_t, 1e-6)
                log_speed[i] = float(np.linalg.norm(cur_pos - prev_pos) / seg_dt)
                prev_pos = cur_pos.copy()
                prev_t = now

                sleep_for = (t0 + (i + 1) * dt) - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
        except KeyboardInterrupt:
            print("[fixed_delta] interrupted by user", file=sys.stderr)

        # Report displacement and the steady-state speed (mean of the last 25%).
        final = robot.get_state()
        final_pos = final.ee_pos if final is not None else prev_pos
        disp = final_pos - init_pos
        tail = max(1, n // 4)
        v_ss_meas = float(np.mean(log_speed[-tail:]))
        elapsed = time.monotonic() - t0
        print(
            f"[fixed_delta] net EE displacement = {disp} m "
            f"(|disp| = {np.linalg.norm(disp):.4f} m over {elapsed:.2f} s)."
        )
        print(
            f"[fixed_delta] measured steady-state speed (last 25%) ≈ "
            f"{v_ss_meas:.4f} m/s  vs  expected {v_ss_expected:.4f} m/s "
            f"(diff explained by the {args.rate:.0f} Hz vs 1 kHz re-anchor rate)."
        )
        print(f"[fixed_delta] final EE rot deviation from start = {log_rotdev[-1]:.4f} rad.")
        print(f"[fixed_delta] achieved loop rate ≈ {n / max(elapsed, 1e-6):.1f} Hz.")

        # Return to the start pose so the arm doesn't drift away after the test.
        if not args.no_return:
            robot.set_ee_target(init_pos, anchor_quat_wxyz)
            time.sleep(0.5)
            settled = robot.get_state()
            if settled is not None:
                back_err = float(np.linalg.norm(settled.ee_pos - init_pos))
                print(f"[fixed_delta] returned to start: |ee_pos - start| = {back_err:.4f} m.")

        if log_path is not None:
            _write_csv(log_path, log_t, log_pos, init_pos, log_quat, log_rotdev, log_speed)
            _write_meta_json(
                log_path,
                kp_pos=kp_pos,
                kp_ori=kp_ori,
                kd_pos_critical=kd_pos,
                kd_ori_critical=kd_ori,
                delta=delta,
                duration_s=float(args.duration),
                refresh_rate_hz_target=float(args.rate),
                refresh_rate_hz_achieved=float(n / max(elapsed, 1e-6)),
                n_ticks=int(n),
                start_ee_pos=init_pos,
                start_ee_quat_wxyz=anchor_quat_wxyz,
                final_disp=disp,
                v_ss_expected_mps=float(v_ss_expected),
                v_ss_measured_mps=float(v_ss_meas),
                final_rot_dev_rad=float(log_rotdev[-1]),
            )
    return 0


def _write_csv(
    path: Path,
    t: np.ndarray,
    pos: np.ndarray,
    init_pos: np.ndarray,
    quat_wxyz: np.ndarray,
    rotdev: np.ndarray,
    speed: np.ndarray,
) -> None:
    disp = pos - init_pos[None, :]
    columns = [
        "t_s", "x", "y", "z",
        "disp_x", "disp_y", "disp_z",
        "qw", "qx", "qy", "qz",
        "rot_dev_rad", "speed_mps",
    ]
    data = np.column_stack((t, pos, disp, quat_wxyz, rotdev, speed))
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, data, delimiter=",", header=",".join(columns), comments="", fmt="%.9f")
    print(f"[fixed_delta] wrote CSV: {path}")


def _write_meta_json(
    csv_path: Path,
    *,
    kp_pos: float,
    kp_ori: float,
    kd_pos_critical: float,
    kd_ori_critical: float,
    delta: np.ndarray,
    duration_s: float,
    refresh_rate_hz_target: float,
    refresh_rate_hz_achieved: float,
    n_ticks: int,
    start_ee_pos: np.ndarray,
    start_ee_quat_wxyz: np.ndarray,
    final_disp: np.ndarray,
    v_ss_expected_mps: float,
    v_ss_measured_mps: float,
    final_rot_dev_rad: float,
) -> None:
    """Write a sidecar ``.json`` next to ``csv_path`` capturing the run setup
    and outcome.  Convention matches panda_control's existing chirp logs
    (``<run>.csv`` + ``<run>.json``).  Downstream sim2real diff tools should
    read this to identify the gains / delta / start pose without parsing the
    filename.
    """
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
            "delta_per_tick_m": [float(v) for v in delta],
            "duration_s": float(duration_s),
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
            "v_ss_expected_mps": float(v_ss_expected_mps),
            "v_ss_measured_mps_last_25pct": float(v_ss_measured_mps),
            "final_rot_dev_rad": float(final_rot_dev_rad),
        },
        "source": {
            "script": "panda_control/examples/fixed_delta_pose_test.py",
            "csv": csv_path.name,
        },
    }
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(meta, indent=2))
    print(f"[fixed_delta] wrote meta: {json_path}")


if __name__ == "__main__":
    raise SystemExit(main())
