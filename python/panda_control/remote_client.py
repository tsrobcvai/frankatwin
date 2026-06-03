"""PC-side client. Same API surface as LocalPandaController.

Talks to panda_control.daemon over ZMQ. REQ socket for synchronous commands;
SUB socket for the 100 Hz state stream which is cached locally so get_state()
is non-blocking on the network.

Drop-in note: anywhere the policy code imports LocalPandaController it can
import RemotePandaClient instead; the method signatures are identical.
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

from panda_control.config import RobotConfig
from panda_control.local_controller import RobotState

logger = logging.getLogger(__name__)

DEFAULT_REQ_TIMEOUT_S = 5.0
MOVE_TO_REQ_TIMEOUT_S = 60.0
SOCKET_LINGER_MS = 200


class RemotePandaClient:
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

    def __enter__(self) -> "RemotePandaClient":
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
        speed_factor: Optional[float] = None,
    ) -> None:
        q = np.asarray(q_target, dtype=np.float64).reshape(-1)
        if q.shape != (7,):
            raise ValueError(f"q_target must have shape (7,), got {q.shape}")
        payload: Dict[str, Any] = {"op": "move_to_q", "q": q.tolist()}
        if speed_factor is not None:
            payload["speed_factor"] = float(speed_factor)
        self._call(payload, timeout_s=MOVE_TO_REQ_TIMEOUT_S)

    def move_to_pose(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        duration: Optional[float] = None,
    ) -> None:
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
        if duration is not None:
            payload["duration"] = float(duration)
        self._call(payload, timeout_s=MOVE_TO_REQ_TIMEOUT_S)

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
    )
