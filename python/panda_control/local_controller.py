"""LocalPandaController: NUC-side, same-machine controller wrapper.

Responsibilities:
- Own the POSIX shm segment (`shm_open(name, O_CREAT)`).
- Start the C++ `osc_shm` subprocess as the 1 kHz controller.
- Provide `set_ee_target`, `set_gains`, `enable`, `disable`, `get_state`.
- For one-shot resets, stop `osc_shm`, spawn `move_to`, wait, restart `osc_shm`.
  Mutual exclusion is required because libfranka grants only one TCP session
  to the FCI port at a time.

This class is intended to be used either standalone on the NUC for local
testing, or composed inside `panda_control.daemon.PandaDaemon` for the
PC-driven remote use case.
"""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from panda_control.config import RobotConfig
from panda_control.shm_layout import (
    PANDA_SHM_STATE_FRAMES,
    STATE_FRAME_DTYPE,
    SharedMemoryAccess,
)

logger = logging.getLogger(__name__)

_OSC_STARTUP_TIMEOUT_S = 10.0
_OSC_SHUTDOWN_TIMEOUT_S = 3.0
_MOVE_TO_TIMEOUT_S = 30.0
# After osc_shm's PID is alive, require state_head to advance this many frames
# before considering it "ready".  state_head ticks at 1 kHz so 5 frames = 5 ms;
# the meaningful wait is the libfranka session setup + first control tick
# which can take 0.5-2 s, not the 5 ms.
_OSC_READY_STATE_HEAD_ADVANCE = 5


def _wxyz_from(quat: np.ndarray) -> np.ndarray:
    """Coerce a (4,) array into wxyz; raises ValueError on shape mismatch."""
    q = np.asarray(quat, dtype=np.float64).reshape(-1)
    if q.shape != (4,):
        raise ValueError(f"quat must have 4 elements, got shape {quat.shape}")
    n = np.linalg.norm(q)
    if n < 1e-9:
        raise ValueError("quat has near-zero norm")
    return q / n


@dataclass
class RobotState:
    """Plain-data snapshot of the latest robot state frame."""

    timestamp_s: float
    q: np.ndarray         # (7,)
    dq: np.ndarray        # (7,)
    ee_pos: np.ndarray    # (3,)
    ee_quat: np.ndarray   # (4,) wxyz
    tau: np.ndarray       # (7,)
    seq: int

    @classmethod
    def from_frame(cls, frame: np.ndarray) -> "RobotState":
        return cls(
            timestamp_s=float(frame["timestamp_s"]),
            q=np.array(frame["q"], dtype=np.float64),
            dq=np.array(frame["dq"], dtype=np.float64),
            ee_pos=np.array(frame["ee_pos"], dtype=np.float64),
            ee_quat=np.array(frame["ee_quat"], dtype=np.float64),
            tau=np.array(frame["tau"], dtype=np.float64),
            seq=int(frame["seq"]),
        )


class LocalPandaController:
    """Owns the shm segment and the C++ controller subprocess.

    Typical usage:

        cfg = load_config()
        with LocalPandaController(cfg) as robot:
            robot.set_gains(kp_pos=200, kp_ori=20)
            robot.set_ee_target(np.array([0.5, 0.0, 0.4]),
                                np.array([1.0, 0.0, 0.0, 0.0]))
            state = robot.get_state()
    """

    def __init__(
        self,
        cfg: RobotConfig,
        *,
        autostart: bool = True,
        verbose: bool = False,
    ) -> None:
        self.cfg = cfg
        self.verbose = verbose
        self._shm: Optional[SharedMemoryAccess] = None
        self._proc: Optional[subprocess.Popen] = None
        self._proc_lock = threading.Lock()

        self._osc_shm_bin = cfg.paths.build_dir / "osc_shm"
        self._move_to_bin = cfg.paths.build_dir / "move_to"
        if not self._osc_shm_bin.is_file():
            raise FileNotFoundError(
                f"osc_shm binary not found at {self._osc_shm_bin}. "
                "Did you run `cmake --build build`?"
            )
        if not self._move_to_bin.is_file():
            raise FileNotFoundError(
                f"move_to binary not found at {self._move_to_bin}"
            )

        # Create and own the shm segment up front. The C++ child only opens it.
        self._shm = SharedMemoryAccess(name=cfg.paths.shm_name, create=True)

        if autostart:
            self.start_controller()

    # ------------------------------------------------------------------ lifecycle
    def start_controller(self) -> None:
        with self._proc_lock:
            if self._proc is not None and self._proc.poll() is None:
                return
            args = [
                str(self._osc_shm_bin),
                self.cfg.robot.ip,
                "--shm-name",
                self.cfg.paths.shm_name,
            ]
            # Register an EE payload (e.g. mounted camera) for gravity comp.
            load = getattr(self.cfg, "load", None)
            if load is not None and load.mass > 0.0:
                args += ["--load-mass", f"{load.mass:.6f}"]
                args += ["--load-com", *[f"{v:.6f}" for v in load.com]]
                args += ["--load-inertia", *[f"{v:.9f}" for v in load.inertia]]
            if self.verbose:
                logger.info("starting osc_shm: %s", " ".join(args))
            self._proc = subprocess.Popen(
                args,
                stdout=subprocess.PIPE if not self.verbose else None,
                stderr=subprocess.PIPE if not self.verbose else None,
                start_new_session=True,
            )
        self._wait_until_running(_OSC_STARTUP_TIMEOUT_S)

    def stop_controller(self) -> None:
        with self._proc_lock:
            proc = self._proc
            self._proc = None
        if proc is None:
            return
        try:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=_OSC_SHUTDOWN_TIMEOUT_S)
                except subprocess.TimeoutExpired:
                    logger.warning("osc_shm did not exit on SIGINT, killing")
                    proc.kill()
                    proc.wait(timeout=1.0)
        finally:
            self._mark_pid_inactive()

    def close(self) -> None:
        try:
            self.stop_controller()
        finally:
            if self._shm is not None:
                self._shm.close(unlink=True)
                self._shm = None

    def __enter__(self) -> "LocalPandaController":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    # ------------------------------------------------------------------ helpers
    @property
    def _view(self):
        if self._shm is None:
            raise RuntimeError("LocalPandaController is closed")
        return self._shm.view

    def _wait_until_running(self, timeout: float) -> None:
        """Wait for osc_shm to actually be producing state frames.

        Two-stage check:
          1. controller_pid is set + that PID is alive (osc_shm process started)
          2. state_head advances by >= _OSC_READY_STATE_HEAD_ADVANCE frames
             (libfranka session is up AND the 1 kHz loop is publishing fresh
             state into shm)

        Stage 2 closes the race after move_to_q: previously this returned as
        soon as the C++ process registered, but libfranka session setup +
        first control tick still take 0.5-2 s, during which the daemon's PUB
        has nothing fresh to send.  A wait_for_state immediately after a
        reset would then time out spuriously.
        """
        deadline = time.monotonic() + timeout
        seen_pid = False
        baseline_head: Optional[int] = None
        while time.monotonic() < deadline:
            pid = self._view.controller_pid
            if not seen_pid:
                if pid != 0 and self._is_pid_alive(pid):
                    seen_pid = True
                    baseline_head = self._view.state_head
            else:
                current_head = self._view.state_head
                if (
                    baseline_head is not None
                    and current_head >= baseline_head + _OSC_READY_STATE_HEAD_ADVANCE
                ):
                    return
            if self._proc is not None and self._proc.poll() is not None:
                stderr = b""
                try:
                    stderr = self._proc.stderr.read() if self._proc.stderr else b""
                except Exception:
                    pass
                raise RuntimeError(
                    f"osc_shm exited with code {self._proc.returncode}. "
                    f"stderr: {stderr.decode(errors='replace')[:512]}"
                )
            time.sleep(0.05)
        if not seen_pid:
            raise TimeoutError(
                f"osc_shm did not register pid in shm within {timeout:.1f} s"
            )
        raise TimeoutError(
            f"osc_shm pid registered but state_head did not advance "
            f"{_OSC_READY_STATE_HEAD_ADVANCE} frames within {timeout:.1f} s "
            f"(baseline {baseline_head} -> {self._view.state_head})"
        )

    @staticmethod
    def _is_pid_alive(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except (ProcessLookupError, PermissionError):
            return False
        except OSError:
            return False
        return True

    def _mark_pid_inactive(self) -> None:
        try:
            if self._shm is not None:
                self._view.controller_pid = 0
        except Exception:
            pass

    # ------------------------------------------------------------------ commands
    def set_ee_target(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
    ) -> None:
        """Publish a new EE setpoint. wxyz quaternion."""
        pos = np.asarray(target_pos, dtype=np.float64).reshape(-1)
        if pos.shape != (3,):
            raise ValueError(f"target_pos must have shape (3,), got {pos.shape}")
        quat = _wxyz_from(target_quat)
        # Re-publish gains untouched.
        prev = self._view.read_command()
        self._view.write_command(
            target_pos=pos,
            target_quat=quat,
            kp_pos=float(prev["kp_pos"]),
            kp_ori=float(prev["kp_ori"]),
            kd_pos=float(prev["kd_pos"]),
            kd_ori=float(prev["kd_ori"]),
            error_delta_pos=float(prev["error_delta_pos"]),
            error_delta_rot=float(prev["error_delta_rot"]),
            enabled=bool(prev["enabled"]),
        )

    def set_gains(
        self,
        kp_pos: Optional[float] = None,
        kp_ori: Optional[float] = None,
        kd_pos: Optional[float] = None,
        kd_ori: Optional[float] = None,
        error_delta_pos: Optional[float] = None,
        error_delta_rot: Optional[float] = None,
    ) -> None:
        """Update controller gains. Any None argument keeps the current value."""
        prev = self._view.read_command()
        self._view.write_command(
            target_pos=np.array(prev["target_pos"], dtype=np.float64),
            target_quat=np.array(prev["target_quat"], dtype=np.float64),
            kp_pos=float(prev["kp_pos"] if kp_pos is None else kp_pos),
            kp_ori=float(prev["kp_ori"] if kp_ori is None else kp_ori),
            kd_pos=float(prev["kd_pos"] if kd_pos is None else kd_pos),
            kd_ori=float(prev["kd_ori"] if kd_ori is None else kd_ori),
            error_delta_pos=float(
                prev["error_delta_pos"]
                if error_delta_pos is None
                else error_delta_pos
            ),
            error_delta_rot=float(
                prev["error_delta_rot"]
                if error_delta_rot is None
                else error_delta_rot
            ),
            enabled=bool(prev["enabled"]),
        )

    def enable(self) -> None:
        self._set_enabled(True)

    def disable(self) -> None:
        self._set_enabled(False)

    def _set_enabled(self, value: bool) -> None:
        prev = self._view.read_command()
        self._view.write_command(
            target_pos=np.array(prev["target_pos"], dtype=np.float64),
            target_quat=np.array(prev["target_quat"], dtype=np.float64),
            kp_pos=float(prev["kp_pos"]),
            kp_ori=float(prev["kp_ori"]),
            kd_pos=float(prev["kd_pos"]),
            kd_ori=float(prev["kd_ori"]),
            error_delta_pos=float(prev["error_delta_pos"]),
            error_delta_rot=float(prev["error_delta_rot"]),
            enabled=value,
        )

    # ------------------------------------------------------------------ state
    def get_state(self, k: Optional[int] = None) -> Optional[RobotState]:
        """Return the latest RobotState (or None if no frame yet).

        If `k` is provided, returns a list of up to k most recent frames in
        chronological order.
        """
        if k is None:
            frame = self._view.latest_state()
            if frame is None:
                return None
            return RobotState.from_frame(frame)
        frames = self._view.last_k_states(k)
        return [RobotState.from_frame(f) for f in frames]

    def get_all_state(self) -> List[RobotState]:
        return self.get_state(k=PANDA_SHM_STATE_FRAMES)  # type: ignore[return-value]

    # ------------------------------------------------------------------ reset
    def move_to_q(
        self,
        q_target: np.ndarray,
        speed_factor: Optional[float] = None,
    ) -> None:
        """Blocking joint-space reset using libfranka MotionGenerator.

        Mutually exclusive with osc_shm: this method stops the controller,
        runs move_to, then restarts the controller.
        """
        q = np.asarray(q_target, dtype=np.float64).reshape(-1)
        if q.shape != (7,):
            raise ValueError(f"q_target must have shape (7,), got {q.shape}")
        sf = (
            speed_factor
            if speed_factor is not None
            else self.cfg.reset.joint_speed_factor
        )
        if not (0.0 < sf <= 0.5):
            raise ValueError(f"speed_factor must be in (0, 0.5], got {sf}")
        args = [str(self._move_to_bin), self.cfg.robot.ip, "--q"]
        args += [f"{v:.6f}" for v in q.tolist()]
        args += ["--speed-factor", f"{sf:.4f}"]
        self._run_exclusive(args)

    def move_to_pose(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        duration: Optional[float] = None,
    ) -> None:
        """Blocking task-space reset using libfranka CartesianPose motion type.

        target_quat is wxyz. No external IK; libfranka solves internally.
        """
        pos = np.asarray(target_pos, dtype=np.float64).reshape(-1)
        if pos.shape != (3,):
            raise ValueError(f"target_pos must have shape (3,), got {pos.shape}")
        quat = _wxyz_from(target_quat)
        dur = duration if duration is not None else self.cfg.reset.pose_duration
        if not (1.5 <= dur <= 20.0):
            raise ValueError(f"duration must be in [1.5, 20.0] s, got {dur}")
        args = [
            str(self._move_to_bin),
            self.cfg.robot.ip,
            "--pose",
            f"{pos[0]:.6f}",
            f"{pos[1]:.6f}",
            f"{pos[2]:.6f}",
            f"{quat[0]:.6f}",
            f"{quat[1]:.6f}",
            f"{quat[2]:.6f}",
            f"{quat[3]:.6f}",
            "--duration",
            f"{dur:.3f}",
        ]
        self._run_exclusive(args)

    def _run_exclusive(self, args: List[str]) -> None:
        """Run a one-shot binary that needs exclusive libfranka access."""
        had_controller = (
            self._proc is not None and self._proc.poll() is None
        )
        if had_controller:
            self.stop_controller()
        try:
            if self.verbose:
                logger.info("running %s", " ".join(args))
            res = subprocess.run(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=_MOVE_TO_TIMEOUT_S,
            )
            if res.returncode != 0:
                raise RuntimeError(
                    f"move_to exited with code {res.returncode}\n"
                    f"stdout: {res.stdout.decode(errors='replace')[:512]}\n"
                    f"stderr: {res.stderr.decode(errors='replace')[:512]}"
                )
        finally:
            if had_controller:
                # Re-seed: osc_shm captures the new anchor pose on startup so
                # the impedance setpoint matches where the robot now is.
                self.start_controller()
