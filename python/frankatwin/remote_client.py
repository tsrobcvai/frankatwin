"""PC-side client. Same API surface as LocalController.

Talks to frankatwin.daemon over ZMQ. REQ socket for synchronous commands;
SUB socket for the 100 Hz state stream which is cached locally so get_state()
is non-blocking on the network.

Drop-in note: anywhere the policy code imports LocalController it can
import FrankaTwinClient instead; the method signatures are identical.
"""

from __future__ import annotations

import collections
import json
import logging
import threading
import time
from typing import Any, Deque, Dict, List, Optional

import numpy as np
import zmq

from frankatwin.config import RobotConfig
from frankatwin.local_controller import RobotState

logger = logging.getLogger(__name__)

DEFAULT_REQ_TIMEOUT_S = 5.0
MOVE_TO_REQ_TIMEOUT_S = 60.0
GRIPPER_REQ_TIMEOUT_S = 10.0      # stop / fresh state do a TCP round trip to the hand
GRIPPER_WAIT_S = 20.0             # move/grasp < 2 s; homing ~6 s
GRIPPER_POLL_S = 0.05
SOCKET_LINGER_MS = 200


class FrankaTwinClient:
    def __init__(
        self,
        cfg: RobotConfig,
        *,
        connect_timeout_s: float = 3.0,
        verbose: bool = False,
    ) -> None:
        self.cfg = cfg
        self.verbose = verbose
        self._ctx = zmq.Context.instance()
        self._req = self._ctx.socket(zmq.REQ)
        self._req.setsockopt(zmq.LINGER, SOCKET_LINGER_MS)
        self._req_url = f"tcp://{cfg.network.nuc_host}:{cfg.network.cmd_port}"
        self._sub_url = f"tcp://{cfg.network.nuc_host}:{cfg.network.state_port}"
        self._req.connect(self._req_url)
        logger.info("connected REQ to %s", self._req_url)

        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.LINGER, SOCKET_LINGER_MS)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.connect(self._sub_url)
        logger.info("connected SUB to %s", self._sub_url)

        self._state_cache: Deque[RobotState] = collections.deque(
            maxlen=cfg.network.state_cache
        )
        self._cache_lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._sub_thread = threading.Thread(
            target=self._sub_loop, name="panda_state_sub", daemon=True
        )
        self._sub_thread.start()

        # Quick liveness probe so failure is loud.
        self._ping_or_raise(connect_timeout_s)

    # ----------------------------------------------------------------- lifecycle
    def close(self) -> None:
        self._stop_evt.set()
        try:
            self._sub_thread.join(timeout=1.0)
        except Exception:
            pass
        for s in (self._req, self._sub):
            try:
                s.close(linger=SOCKET_LINGER_MS)
            except Exception:
                pass

    def __enter__(self) -> "FrankaTwinClient":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    # ----------------------------------------------------------------- transport
    def _ping_or_raise(self, timeout: float) -> None:
        try:
            self._call({"op": "ping"}, timeout_s=timeout)
        except Exception as e:
            self.close()
            raise RuntimeError(
                f"cannot reach panda daemon at {self._req_url}: {e}"
            ) from e

    def _call(
        self,
        req: Dict[str, Any],
        timeout_s: float = DEFAULT_REQ_TIMEOUT_S,
    ) -> Dict[str, Any]:
        # ZMQ REQ sockets enforce strict alternation. If the previous send timed
        # out we must rebuild the socket to recover.
        try:
            self._req.send(json.dumps(req).encode("utf-8"), flags=zmq.NOBLOCK)
        except zmq.ZMQError as e:
            self._reset_req_socket()
            raise RuntimeError(f"send failed: {e}") from e
        if not self._req.poll(timeout=int(timeout_s * 1000)):
            self._reset_req_socket()
            raise TimeoutError(
                f"daemon did not reply within {timeout_s:.1f}s for op={req.get('op')}"
            )
        raw = self._req.recv()
        try:
            reply = json.loads(raw.decode("utf-8"))
        except Exception as e:
            raise RuntimeError(f"invalid reply: {e}") from e
        if not isinstance(reply, dict) or not reply.get("ok"):
            err = reply.get("error", "unknown error") if isinstance(reply, dict) else reply
            raise RuntimeError(f"daemon error: {err}")
        return reply

    def _reset_req_socket(self) -> None:
        try:
            self._req.close(linger=0)
        except Exception:
            pass
        self._req = self._ctx.socket(zmq.REQ)
        self._req.setsockopt(zmq.LINGER, SOCKET_LINGER_MS)
        self._req.connect(self._req_url)

    def _sub_loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)
        while not self._stop_evt.is_set():
            events = dict(poller.poll(timeout=250))
            if self._sub not in events:
                continue
            try:
                raw = self._sub.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                continue
            try:
                d = json.loads(raw.decode("utf-8"))
            except Exception:
                continue
            state = _state_from_dict(d)
            with self._cache_lock:
                self._state_cache.append(state)

    # ----------------------------------------------------------------- commands
    def set_ee_target(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
    ) -> None:
        pos = np.asarray(target_pos, dtype=np.float64).reshape(-1)
        quat = np.asarray(target_quat, dtype=np.float64).reshape(-1)
        if pos.shape != (3,):
            raise ValueError(f"target_pos must have shape (3,), got {pos.shape}")
        if quat.shape != (4,):
            raise ValueError(f"target_quat must have shape (4,), got {quat.shape}")
        self._call({
            "op": "set_ee_target",
            "pos": pos.tolist(),
            "quat": quat.tolist(),
        })

    def set_gains(
        self,
        kp_pos: Optional[float] = None,
        kp_ori: Optional[float] = None,
        kd_pos: Optional[float] = None,
        kd_ori: Optional[float] = None,
        error_delta_pos: Optional[float] = None,
        error_delta_rot: Optional[float] = None,
    ) -> None:
        payload: Dict[str, Any] = {"op": "set_gains"}
        if kp_pos is not None:
            payload["kp_pos"] = float(kp_pos)
        if kp_ori is not None:
            payload["kp_ori"] = float(kp_ori)
        if kd_pos is not None:
            payload["kd_pos"] = float(kd_pos)
        if kd_ori is not None:
            payload["kd_ori"] = float(kd_ori)
        if error_delta_pos is not None:
            payload["error_delta_pos"] = float(error_delta_pos)
        if error_delta_rot is not None:
            payload["error_delta_rot"] = float(error_delta_rot)
        self._call(payload)

    def enable(self) -> None:
        self._call({"op": "enable"})

    def disable(self) -> None:
        self._call({"op": "disable"})

    # ----------------------------------------------------------------- state
    def get_state(self, *, fresh: bool = False) -> Optional[RobotState]:
        """Return the most recent cached state frame.

        If `fresh=True`, force a synchronous round-trip to the daemon instead
        of using the SUB-stream cache. Use sparingly; the cache is sufficient
        for policy loops.
        """
        if fresh:
            reply = self._call({"op": "get_state"})
            d = reply.get("state")
            return None if d is None else _state_from_dict(d)
        with self._cache_lock:
            if not self._state_cache:
                return None
            return self._state_cache[-1]

    def get_state_history(self) -> List[RobotState]:
        with self._cache_lock:
            return list(self._state_cache)

    # ----------------------------------------------------------------- reset
    def move_to_q(
        self,
        q_target: np.ndarray,
        q_max_speed: Optional[float] = None,
    ) -> None:
        q = np.asarray(q_target, dtype=np.float64).reshape(-1)
        if q.shape != (7,):
            raise ValueError(f"q_target must have shape (7,), got {q.shape}")
        payload: Dict[str, Any] = {"op": "move_to_q", "q": q.tolist()}
        if q_max_speed is not None:
            payload["q_max_speed"] = float(q_max_speed)
        self._call(payload, timeout_s=MOVE_TO_REQ_TIMEOUT_S)

    def move_to_pose(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        q_max_speed: Optional[float] = None,
    ) -> None:
        """`q_max_speed` is approximate here -- see LocalController.move_to_pose."""
        pos = np.asarray(target_pos, dtype=np.float64).reshape(-1)
        quat = np.asarray(target_quat, dtype=np.float64).reshape(-1)
        if pos.shape != (3,):
            raise ValueError(f"target_pos must have shape (3,), got {pos.shape}")
        if quat.shape != (4,):
            raise ValueError(f"target_quat must have shape (4,), got {quat.shape}")
        payload: Dict[str, Any] = {
            "op": "move_to_pose",
            "pos": pos.tolist(),
            "quat": quat.tolist(),
        }
        if q_max_speed is not None:
            payload["q_max_speed"] = float(q_max_speed)
        self._call(payload, timeout_s=MOVE_TO_REQ_TIMEOUT_S)

    # ----------------------------------------------------------------- gripper
    # Same names as LocalController. The daemon runs the hardware call on its
    # own thread; with wait=True (default) these block until it finished and
    # return its output: {"ok", "cmd", "result", "stopped", "state": {"width",
    # "max_width", "is_grasped", "temperature"}, "seq"}. wait=False returns
    # {"started": true, "seq"} at once; collect with gripper_wait(seq).
    def gripper_homing(self, *, wait: bool = True) -> Dict[str, Any]:
        """Calibrate the finger stroke. Once after power-up or a finger change (~6 s)."""
        return self._gripper_cmd({"op": "gripper_homing"}, wait)

    def gripper_move(
        self, width: float, speed: Optional[float] = None, *, wait: bool = True
    ) -> Dict[str, Any]:
        """Fingers to `width` [m] (position only, no force)."""
        payload: Dict[str, Any] = {"op": "gripper_move", "width": float(width)}
        if speed is not None:
            payload["speed"] = float(speed)
        return self._gripper_cmd(payload, wait)

    def gripper_open(
        self, width: Optional[float] = None, speed: Optional[float] = None, *, wait: bool = True
    ) -> Dict[str, Any]:
        """Open to `width` [m]; default `gripper.max_width` (0.08 = fully open)."""
        w = self.cfg.gripper.max_width if width is None else width
        return self.gripper_move(w, speed, wait=wait)

    def gripper_grasp(
        self,
        width: Optional[float] = None,
        speed: Optional[float] = None,
        force: Optional[float] = None,
        epsilon_inner: Optional[float] = None,
        epsilon_outer: Optional[float] = None,
        *,
        wait: bool = True,
    ) -> Dict[str, Any]:
        """Close on an object: drive towards `width`, squeeze with `force` [N] on stall.

        None = robot.yaml `gripper:` defaults (width -0.01 = past closure, so the
        object sets the resting width; force 70 N). `result` is libfranka's
        within-epsilon verdict, `state.width` where the fingers stopped.
        """
        payload: Dict[str, Any] = {"op": "gripper_grasp"}
        for k, v in (("width", width), ("speed", speed), ("force", force),
                     ("epsilon_inner", epsilon_inner), ("epsilon_outer", epsilon_outer)):
            if v is not None:
                payload[k] = float(v)
        return self._gripper_cmd(payload, wait)

    gripper_close = gripper_grasp

    def gripper_stop(self) -> Dict[str, Any]:
        """Abort the gripper motion in flight."""
        return self._call({"op": "gripper_stop"}, timeout_s=GRIPPER_REQ_TIMEOUT_S)

    def gripper_state(self) -> Dict[str, Any]:
        """Live `{"width", "max_width", "is_grasped", "temperature"}` (one round trip
        to the hand). Raises if a command is running; use gripper_wait first."""
        reply = self._call({"op": "gripper_state", "fresh": True}, timeout_s=GRIPPER_REQ_TIMEOUT_S)
        if reply.get("busy"):
            raise RuntimeError("gripper busy; gripper_wait() before reading a live state")
        return reply["state"]

    def gripper_wait(self, seq: Optional[int] = None, timeout_s: float = GRIPPER_WAIT_S) -> Dict[str, Any]:
        """Block until the running gripper command (or the one with `seq`) finished.

        Returns its output dict; raises RuntimeError if it failed.
        """
        deadline = time.monotonic() + timeout_s
        while True:
            reply = self._call({"op": "gripper_state", "fresh": False})
            last = reply.get("last") or {}
            done = not reply.get("busy") and (seq is None or last.get("seq") == seq)
            if done:
                if not last.get("ok", False):
                    raise RuntimeError(f"gripper {last.get('cmd')} failed: {last.get('error', 'unknown error')}")
                return last
            if time.monotonic() > deadline:
                raise TimeoutError(f"gripper command still running after {timeout_s:.0f}s")
            time.sleep(GRIPPER_POLL_S)

    def _gripper_cmd(self, payload: Dict[str, Any], wait: bool) -> Dict[str, Any]:
        reply = self._call(payload)
        if not wait:
            return {"started": True, "seq": reply.get("seq")}
        return self.gripper_wait(seq=reply.get("seq"))

    # ----------------------------------------------------------------- helpers
    def wait_for_state(self, timeout_s: float = 3.0) -> RobotState:
        """Block until the SUB stream delivers at least one frame."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.get_state()
            if s is not None:
                return s
            time.sleep(0.01)
        raise TimeoutError("no state frame received from daemon")


def _state_from_dict(d: Dict[str, Any]) -> RobotState:
    # ee_linvel/ee_angvel arrive from daemons running shm v2+. Fall back to zeros
    # if talking to an older daemon so a partial deploy degrades instead of crashing.
    return RobotState(
        timestamp_s=float(d["timestamp_s"]),
        q=np.asarray(d["q"], dtype=np.float64),
        dq=np.asarray(d["dq"], dtype=np.float64),
        ee_pos=np.asarray(d["ee_pos"], dtype=np.float64),
        ee_quat=np.asarray(d["ee_quat"], dtype=np.float64),
        tau=np.asarray(d["tau"], dtype=np.float64),
        seq=int(d["seq"]),
        ee_linvel=np.asarray(d.get("ee_linvel", [0.0, 0.0, 0.0]), dtype=np.float64),
        ee_angvel=np.asarray(d.get("ee_angvel", [0.0, 0.0, 0.0]), dtype=np.float64),
        # NaN (not 0) when the daemon predates shm v3 so it can't be mistaken
        # for a real "0 Nm" measurement.
        tau_J=np.asarray(d.get("tau_J", [float("nan")] * 7), dtype=np.float64),
    )
