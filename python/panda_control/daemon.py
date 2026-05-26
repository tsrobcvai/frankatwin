"""NUC-side daemon. Bridges PC ZMQ traffic to the local shm/C++ controller.

Two sockets:
  - REP at tcp://*:cmd_port    -> request/reply (set_ee_target, set_gains,
                                   move_to_*, get_state, enable, disable,
                                   ping, shutdown).
  - PUB at tcp://*:state_port  -> 100 Hz JSON broadcast of the latest state
                                   frame.

Protocol uses plain JSON (UTF-8). Numpy arrays cross as Python lists. The
command path runs at <= 20 Hz so JSON parsing cost is negligible; the state
PUB path is ~1.5 KB/msg @ 100 Hz = 150 KB/s, also fine.

Usage:
    python -m panda_control.daemon [--config /path/to/robot.yaml]
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import threading
import time
from typing import Any, Dict, Optional

import numpy as np
import zmq

from panda_control.config import RobotConfig, load_config
from panda_control.local_controller import LocalPandaController, RobotState

logger = logging.getLogger(__name__)

STATE_PUB_HZ = 100.0
SOCKET_LINGER_MS = 200


# ---------------------------------------------------------------------------
# JSON helpers
# ---------------------------------------------------------------------------
def _state_to_dict(state: RobotState) -> Dict[str, Any]:
    return {
        "timestamp_s": state.timestamp_s,
        "q": state.q.tolist(),
        "dq": state.dq.tolist(),
        "ee_pos": state.ee_pos.tolist(),
        "ee_quat": state.ee_quat.tolist(),
        "tau": state.tau.tolist(),
        "seq": state.seq,
    }


# ---------------------------------------------------------------------------
# Daemon
# ---------------------------------------------------------------------------
class PandaDaemon:
    def __init__(self, cfg: RobotConfig, *, verbose: bool = False) -> None:
        self.cfg = cfg
        self.verbose = verbose
        self._stop_evt = threading.Event()
        self._ctx = zmq.Context.instance()
        self._rep = self._ctx.socket(zmq.REP)
        self._pub = self._ctx.socket(zmq.PUB)
        for s in (self._rep, self._pub):
            s.setsockopt(zmq.LINGER, SOCKET_LINGER_MS)
        self._rep.bind(f"tcp://*:{cfg.network.cmd_port}")
        self._pub.bind(f"tcp://*:{cfg.network.state_port}")
        logger.info(
            "ZMQ bound: REP tcp://*:%d, PUB tcp://*:%d",
            cfg.network.cmd_port,
            cfg.network.state_port,
        )

        self.controller = LocalPandaController(cfg, autostart=True, verbose=verbose)
        # We must serialize ops since move_to* restarts the C++ child and we
        # cannot let another command land between stop/start.
        self._op_lock = threading.Lock()
        self._pub_thread: Optional[threading.Thread] = None

    # ---------------------------------------------------------------- run loop
    def run(self) -> None:
        self._pub_thread = threading.Thread(
            target=self._state_pub_loop, name="panda_state_pub", daemon=True
        )
        self._pub_thread.start()

        # Poll the REP socket so we can react to SIGINT/SIGTERM.
        poller = zmq.Poller()
        poller.register(self._rep, zmq.POLLIN)
        logger.info("daemon ready, awaiting commands")
        try:
            while not self._stop_evt.is_set():
                events = dict(poller.poll(timeout=250))
                if self._rep in events:
                    raw = self._rep.recv()
                    reply = self._dispatch(raw)
                    self._rep.send(reply)
        finally:
            self._shutdown()

    def request_stop(self) -> None:
        self._stop_evt.set()

    def _shutdown(self) -> None:
        logger.info("daemon shutting down")
        try:
            self.controller.close()
        except Exception:
            logger.exception("controller.close() failed")
        for s in (self._rep, self._pub):
            try:
                s.close(linger=SOCKET_LINGER_MS)
            except Exception:
                pass
        # We intentionally do NOT term the global context here; pyzmq leaves
        # this up to the embedding process (so tests can re-bind cleanly).

    # ---------------------------------------------------------------- state pub
    def _state_pub_loop(self) -> None:
        dt = 1.0 / STATE_PUB_HZ
        last_seq = -1
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            state = self.controller.get_state()
            if state is not None and state.seq != last_seq:
                last_seq = state.seq
                msg = json.dumps(_state_to_dict(state)).encode("utf-8")
                try:
                    self._pub.send(msg, flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass  # subscriber slow; drop
            elapsed = time.monotonic() - t0
            sleep_for = dt - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    # ---------------------------------------------------------------- dispatch
    def _dispatch(self, raw: bytes) -> bytes:
        try:
            req = json.loads(raw.decode("utf-8"))
        except Exception as e:
            return self._err(f"invalid JSON: {e}")
        op = req.get("op") if isinstance(req, dict) else None
        if not isinstance(op, str):
            return self._err("missing 'op' field")

        handler_name = f"_op_{op}"
        handler = getattr(self, handler_name, None)
        if handler is None:
            return self._err(f"unknown op: {op}")
        try:
            with self._op_lock:
                result = handler(req)
        except Exception as e:
            logger.exception("op %s failed", op)
            return self._err(f"{op}: {e}")
        if result is None:
            result = {}
        result["ok"] = True
        return json.dumps(result).encode("utf-8")

    @staticmethod
    def _err(msg: str) -> bytes:
        return json.dumps({"ok": False, "error": msg}).encode("utf-8")

    # ---------------------------------------------------------------- handlers
    def _op_ping(self, _req: Dict[str, Any]) -> Dict[str, Any]:
        return {"pid": self.controller._view.controller_pid}

    def _op_set_ee_target(self, req: Dict[str, Any]) -> None:
        pos = np.asarray(req["pos"], dtype=np.float64)
        quat = np.asarray(req["quat"], dtype=np.float64)
        self.controller.set_ee_target(pos, quat)

    def _op_set_gains(self, req: Dict[str, Any]) -> None:
        self.controller.set_gains(
            kp_pos=req.get("kp_pos"),
            kp_ori=req.get("kp_ori"),
            kd_pos=req.get("kd_pos"),
            kd_ori=req.get("kd_ori"),
            error_delta_pos=req.get("error_delta_pos"),
            error_delta_rot=req.get("error_delta_rot"),
        )

    def _op_enable(self, _req: Dict[str, Any]) -> None:
        self.controller.enable()

    def _op_disable(self, _req: Dict[str, Any]) -> None:
        self.controller.disable()

    def _op_get_state(self, _req: Dict[str, Any]) -> Dict[str, Any]:
        state = self.controller.get_state()
        if state is None:
            return {"state": None}
        return {"state": _state_to_dict(state)}

    def _op_move_to_q(self, req: Dict[str, Any]) -> None:
        q = np.asarray(req["q"], dtype=np.float64)
        sf = req.get("speed_factor")
        self.controller.move_to_q(q, speed_factor=sf)

    def _op_move_to_pose(self, req: Dict[str, Any]) -> None:
        pos = np.asarray(req["pos"], dtype=np.float64)
        quat = np.asarray(req["quat"], dtype=np.float64)
        duration = req.get("duration")
        self.controller.move_to_pose(pos, quat, duration=duration)

    def _op_shutdown(self, _req: Dict[str, Any]) -> Dict[str, Any]:
        # Signal the main loop to stop after we ack the request.
        threading.Thread(target=self.request_stop, daemon=True).start()
        return {"shutting_down": True}


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(prog="panda_control.daemon")
    parser.add_argument(
        "--config", "-c", type=str, default=None, help="path to robot.yaml"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="verbose logging"
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cfg = load_config(args.config)
    logger.info("loaded config from %s", cfg.source_path)
    daemon = PandaDaemon(cfg, verbose=args.verbose)

    def _on_signal(_signo, _frame):
        logger.info("received signal, stopping")
        daemon.request_stop()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    daemon.run()


if __name__ == "__main__":
    main()
