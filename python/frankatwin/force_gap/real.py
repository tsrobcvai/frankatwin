"""Guided real-robot collection through the existing FrankaTwin daemon."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np

from frankatwin.config import load_config
from frankatwin.remote_client import FrankaTwinClient
from frankatwin.ring_log import MERGED_COLUMNS, WRENCH_COLUMNS
from frankatwin.shm_layout import WRENCH_STATE_FIELDS
from .protocol import Protocol
from .math import quaternion_angle, validate_transform
from .data import save_bundle, write_json, derive_frames, load_bundle


def confirm(prompt, token):
    answer = input(f"{prompt}\nType {token} to continue: ").strip()
    if answer != token:
        raise RuntimeError("operator cancelled")


def require_wrench_state(state):
    if state is None:
        raise RuntimeError("no state from the controller")
    for name in WRENCH_STATE_FIELDS:
        if not np.all(np.isfinite(getattr(state, name))):
            raise RuntimeError(
                f"{name} unavailable: rebuild osc_shm and update both daemon and client to shm v4"
            )
    validate_transform(state.EE_T_K.reshape(4, 4, order="F"))
    validate_transform(state.F_T_EE.reshape(4, 4, order="F"))


def normalize_log(merged, wrench, events, protocol, metadata):
    """Join the full-rate tables by sequence ID; never align by arrival time."""
    col = {n: i for i, n in enumerate(MERGED_COLUMNS)}
    wc = {n: i for i, n in enumerate(WRENCH_COLUMNS)}
    if (
        len(merged) < 3
        or len(merged) != len(wrench)
        or not np.array_equal(merged[:, col["seq"]], wrench[:, wc["seq"]])
    ):
        raise ValueError(
            "merged/wrench tables are empty or have different sequence IDs"
        )
    fields = {
        name: (
            wrench[:, wc[name]]
            if width == 1
            else np.column_stack([wrench[:, wc[f"{name}_{i}"]] for i in range(width)])
        )
        for name, width in WRENCH_STATE_FIELDS.items()
    }
    stamps = wrench[:, wc["timestamp_s"]]
    if np.any(np.diff(stamps) <= 0):
        raise ValueError("controller timestamps are not strictly increasing")
    for name, value in fields.items():
        if not np.all(np.isfinite(value)):
            raise ValueError(f"nonfinite telemetry: {name}")
    for name in (
        "F_T_EE",
        "EE_T_K",
        "m_ee",
        "m_load",
        "m_total",
        "F_x_Cload",
        "I_load",
        "F_x_Ctotal",
        "I_total",
    ):
        if not np.allclose(fields[name], fields[name][0], rtol=1e-8, atol=1e-9):
            raise ValueError(f"{name} changed during collection")
        expected = (
            np.asarray(metadata[name]).flatten(order="F")
            if name in ("F_T_EE", "EE_T_K") and name in metadata
            else metadata.get("load_model", {}).get(name)
        )
        if expected is not None and not np.allclose(
            fields[name][0], expected, rtol=1e-6, atol=1e-8
        ):
            raise ValueError(f"{name} differs from pre-grasp configuration")
    target_idx = merged[:, col["target_idx"]].astype(int)
    phase = np.zeros(len(merged), int)
    eligible = np.zeros(len(merged), bool)
    for idx, event in enumerate(events):
        mask = target_idx == idx
        phase[mask] = event["phase"]
        eligible[mask] = event["settled"]
    arrays = {
        "t_s": stamps - stamps[0],
        "timestamp_s": stamps,
        "seq": merged[:, col["seq"]].astype(np.int64),
        "phase": phase,
        "settled": eligible,
        "q": np.column_stack([merged[:, col[f"q{i}"]] for i in range(1, 8)]),
        "dq": np.column_stack([merged[:, col[f"dq{i}"]] for i in range(1, 8)]),
        "ee_pos": np.column_stack([merged[:, col[f"x_{a}"]] for a in "xyz"]),
        "ee_quat": np.column_stack([merged[:, col[f"quat_{a}"]] for a in "wxyz"]),
        "target_pos": np.column_stack([merged[:, col[f"x_des_{a}"]] for a in "xyz"]),
        "target_quat": np.column_stack(
            [merged[:, col[f"quat_des_{a}"]] for a in "wxyz"]
        ),
        "wrench_O_K": fields["O_F_ext_hat_K"].copy(),
        **fields,
    }
    arrays["settled"] &= np.max(np.abs(arrays["dq"]), axis=1) < 0.04
    ee_t_k = validate_transform(fields["EE_T_K"][0].reshape(4, 4, order="F"))
    derive_frames(arrays, protocol, ee_t_k)
    if metadata["empty"]:
        arrays["gravity_O_K"][:] = 0.0
        arrays["gravity_K_K"][:] = 0.0
    arrays["all_zero_estimate"] = np.all(
        np.abs(fields["O_F_ext_hat_K"]) < 1e-12, axis=1
    )
    metadata["frame_rotation_rmse"] = np.sqrt(
        np.mean((arrays["wrench_K_K"] - fields["K_F_ext_hat_K"]) ** 2, axis=0)
    ).tolist()
    metadata["zero_estimate_fraction"] = float(arrays["all_zero_estimate"].mean())
    return arrays


def collect(
    args,
    client_factory=FrankaTwinClient,
    prompt=confirm,
    clock=time.monotonic,
    sleep=time.sleep,
):
    protocol = Protocol.load(args.protocol)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    write_json(out / "protocol.json", protocol.to_dict())
    if args.dry_run:
        t = np.arange(0, protocol.duration + 1e-8, 1 / protocol.control_hz)
        samples = [protocol.sample(x, [0.4, 0, 0.4], [0, 1, 0, 0]) for x in t]
        np.savez_compressed(
            out / "preview_targets.npz",
            t_s=t,
            pos=np.array([x[0] for x in samples]),
            quat=np.array([x[1] for x in samples]),
            phase=np.array([x[2] for x in samples]),
        )
        write_json(
            out / "dry_run.json",
            {
                "robot_connected": False,
                "duration_s": protocol.duration,
                "protocol_sha256": protocol.digest,
            },
        )
        print(
            f"Dry run: {protocol.duration:.1f}s, {len(t)} targets. No robot connection. {out}"
        )
        return
    cfg = load_config(args.config)
    baseline = None
    if args.baseline:
        _, baseline = load_bundle(args.baseline)
        if (
            not baseline.get("empty")
            or baseline["protocol_sha256"] != protocol.digest
            or not baseline.get("completed")
        ):
            raise ValueError(
                "baseline must be a completed empty-gripper run of the same protocol"
            )
    metadata = {
        "source": "real",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "protocol": protocol.to_dict(),
        "protocol_sha256": protocol.digest,
        "empty": args.empty,
        "baseline": None if not args.baseline else str(Path(args.baseline).resolve()),
        "sensor": "libfranka O_F_ext_hat_K",
        "object_in_compensation_model": False,
        "completed": False,
        "exception": None,
        "ring_log": None,
    }
    write_json(out / "session.json", metadata)
    with client_factory(cfg) as robot:
        state = robot.get_state(fresh=True)
        require_wrench_state(state)
        prompt(
            "Remove any held object and clear the arm workspace. The arm will reset to the protocol q_init at 0.2 rad/s.",
            "RESET",
        )
        robot.move_to_q(np.asarray(protocol.q_init), q_max_speed=0.2)
        state = robot.get_state(fresh=True)
        require_wrench_state(state)
        if np.max(np.abs(state.q - np.array(protocol.q_init))) > 0.03:
            raise RuntimeError("joint reset did not reach q_init")
        anchor_pos, anchor_quat = state.ee_pos.copy(), state.ee_quat.copy()
        f_t_ee = state.F_T_EE.reshape(4, 4, order="F")
        ee_t_k = state.EE_T_K.reshape(4, 4, order="F")
        if baseline:
            for name, value in [("F_T_EE", f_t_ee), ("EE_T_K", ee_t_k)]:
                if not np.allclose(value, baseline[name], atol=1e-6):
                    raise ValueError(f"{name} differs from baseline")
            for name in (
                "m_ee",
                "m_load",
                "m_total",
                "F_x_Cload",
                "I_load",
                "F_x_Ctotal",
                "I_total",
            ):
                if not np.allclose(
                    getattr(state, name),
                    baseline["load_model"][name],
                    rtol=1e-6,
                    atol=1e-8,
                ):
                    raise ValueError(
                        f"configured load model {name} differs from the empty baseline"
                    )
            anchor_pos = np.asarray(baseline["anchor_pos"])
            anchor_quat = np.asarray(baseline["anchor_quat"])
        metadata.update(
            anchor_pos=anchor_pos.tolist(),
            anchor_quat=anchor_quat.tolist(),
            F_T_EE=f_t_ee.tolist(),
            EE_T_K=ee_t_k.tolist(),
            load_model={
                name: np.asarray(getattr(state, name)).tolist()
                for name in (
                    "m_ee",
                    "m_load",
                    "m_total",
                    "F_x_Cload",
                    "I_load",
                    "F_x_Ctotal",
                    "I_total",
                )
            },
        )
        robot.set_gains(
            kp_pos=protocol.kp_pos,
            kp_ori=protocol.kp_ori,
            kd_pos=2 * np.sqrt(protocol.kp_pos),
            kd_ori=2 * np.sqrt(protocol.kp_ori),
            error_delta_pos=0.04,
        )
        robot.set_ee_target(anchor_pos, anchor_quat)
        robot.enable()
        print("Current controller load model:", json.dumps(metadata["load_model"]))
        prompt(
            "Verify that the configured tool/load describes the EMPTY gripper and its mounted equipment. The test block must be additional to that model; this script preserves the model.",
            "MODEL",
        )
        prompt(
            "The gripper will open to 80 mm. Keep hands clear of its fingers.", "OPEN"
        )
        robot.gripper_open(width=0.08, speed=0.03)
        if args.empty:
            prompt(
                "Leave the gripper EMPTY. It will close to the same width used by the block.",
                "EMPTY",
            )
            robot.gripper_move(protocol.dimensions_m[1], speed=0.02)
        else:
            print(
                f"Place the {protocol.dimensions_m} m block with its 100 mm axis along EE +Z. "
                f"Use a physical placement jig/mark so COM in EE is {protocol.ee_com_m} m. "
                "Finger width alone cannot verify insertion depth or COM."
            )
            prompt(
                "Support the block at the placement mark and clear the finger pinch area. Close and grasp next.",
                "GRASP",
            )
            result = robot.gripper_grasp(
                width=protocol.dimensions_m[1],
                speed=0.02,
                force=protocol.grasp_force_n,
                epsilon_inner=protocol.gripper_width_tolerance_m,
                epsilon_outer=protocol.gripper_width_tolerance_m,
            )
            if not result.get("result", False):
                raise RuntimeError("gripper reported unsuccessful grasp")
        gripper = robot.gripper_state()
        if (
            abs(gripper["width"] - protocol.dimensions_m[1])
            > protocol.gripper_width_tolerance_m
        ):
            raise RuntimeError(
                f'gripper width {gripper["width"]} inconsistent with the block width'
            )
        if not args.empty and not gripper.get("is_grasped", False):
            raise RuntimeError("gripper is not holding the block")
        metadata["gripper_before"] = gripper
        prompt(
            f"Remove the placement support and your hands. The arm will rotate through {len(protocol.rotation_waypoints_deg)} poses for {protocol.duration:.1f}s, then hold its final target.",
            "RUN",
        )
        events = []
        recording = False
        completed = False
        failure = None
        try:
            robot.log_start(path=out / "ring.csv")
            recording = True
            start = clock()
            last_change = start
            last_seq = None
            next_status = start
            while True:
                elapsed = clock() - start
                state = robot.get_state()
                require_wrench_state(state)
                if state.seq != last_seq:
                    last_change = clock()
                    last_seq = state.seq
                if clock() - last_change > 0.25:
                    raise RuntimeError("state stream stale for more than 250 ms")
                pos, quat, phase, settled = protocol.sample(
                    elapsed, anchor_pos, anchor_quat
                )
                if (
                    np.linalg.norm(state.ee_pos - pos) > 0.04
                    or quaternion_angle(state.ee_quat, quat) > 0.4
                ):
                    raise RuntimeError("motion tracking error exceeded 40 mm / 0.4 rad")
                head = robot.set_ee_target(pos, quat)
                events.append(
                    {
                        "elapsed_s": elapsed,
                        "phase": phase,
                        "settled": bool(settled),
                        "head": int(head),
                    }
                )
                if clock() >= next_status:
                    status = {
                        "elapsed_s": elapsed,
                        "phase": phase,
                        "seq": int(state.seq),
                        "O_F_ext_hat_K": state.O_F_ext_hat_K.tolist(),
                    }
                    write_json(out / "progress.json", status)
                    print("[PROGRESS] " + json.dumps(status), flush=True)
                    next_status = clock() + 1.0
                if elapsed >= protocol.duration:
                    completed = True
                    break
                # Wall-time evaluation avoids slowing the protocol when a packet is late.
                next_tick = (
                    np.floor(elapsed * protocol.control_hz) + 1
                ) / protocol.control_hz
                sleep(max(0.0, start + next_tick - clock()))
        except BaseException as exc:
            failure = exc
            metadata["exception"] = f"{type(exc).__name__}: {exc}"
            # Request a hold at the latest measured pose; never open the loaded gripper on failure.
            try:
                state = robot.get_state(fresh=True)
                if state is not None:
                    robot.set_ee_target(state.ee_pos, state.ee_quat)
            except Exception as hold_exc:
                metadata["hold_error"] = str(hold_exc)
        finally:
            write_json(out / "events.json", events)
            if recording:
                try:
                    log = robot.log_stop(save=False)
                    metadata["ring_log"] = log
                    robot.log_save(out / "ring.csv")
                    merged = robot.log_fetch("merged")
                    wrench = robot.log_fetch("wrench")
                    np.savetxt(
                        out / "wrench_raw.csv",
                        wrench,
                        delimiter=",",
                        header=",".join(WRENCH_COLUMNS),
                        comments="",
                        fmt="%.12g",
                    )
                    arrays = normalize_log(merged, wrench, events, protocol, metadata)
                    metadata["completed"] = bool(
                        completed
                        and log.get("dropped_frames", 0) == 0
                        and log.get("resets", 0) == 0
                    )
                    metadata["gripper_after"] = robot.gripper_state()
                    if not args.empty and not metadata["gripper_after"].get(
                        "is_grasped", False
                    ):
                        metadata["completed"] = False
                        metadata["grasp_lost"] = True
                    save_bundle(out, arrays, metadata)
                except Exception as save_exc:
                    metadata["save_error"] = str(save_exc)
                    if failure is None:
                        failure = save_exc
            write_json(out / "session.json", metadata)
        if failure is not None:
            raise failure
        if not metadata["completed"]:
            raise RuntimeError(
                "collection failed its completeness checks; inspect session.json"
            )
        print(
            f"[RESULT] Saved {out}. The arm holds its final target; support the block before opening the gripper."
        )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--config", type=Path)
    p.add_argument("--protocol", type=Path)
    p.add_argument("--empty", action="store_true")
    p.add_argument("--baseline", type=Path)
    p.add_argument("--dry-run", action="store_true")
    collect(p.parse_args())


if __name__ == "__main__":
    main()
