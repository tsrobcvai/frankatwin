"""NUC-side daemon. Bridges PC ZMQ traffic to the local shm/C++ controller.

Two sockets:
  - REP at tcp://*:cmd_port    -> request/reply (set_ee_target, set_gains,
                                   move_to_*, get_state, enable, disable,
                                   gripper_*, ping, shutdown).
  - PUB at tcp://*:state_port  -> 100 Hz JSON broadcast of the latest state
                                   frame.

Protocol uses plain JSON (UTF-8). Numpy arrays cross as Python lists. The
command path runs at <= 20 Hz so JSON parsing cost is negligible; the state
PUB path is ~1.5 KB/msg @ 100 Hz = 150 KB/s, also fine.

Usage:
    python -m frankatwin.daemon [--config /path/to/robot.yaml]
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

from frankatwin.config import RobotConfig, load_config
from frankatwin.local_controller import LocalController, RobotState

logger = logging.getLogger(__name__)

STATE_PUB_HZ = 100.0
SOCKET_LINGER_MS = 200
# How often the watchdog checks that osc_shm is still alive and restarts it if
# not. osc_shm can die mid-run on a libfranka reflex / RT overrun; without a
# restart the daemon keeps ACKing commands into a dead controller (robot stops
# moving while the PC still gets ok replies).
WATCHDOG_POLL_S = 0.5


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
        "ee_linvel": state.ee_linvel.tolist(),  # base frame m/s (shm v2+)
        "ee_angvel": state.ee_angvel.tolist(),  # base frame rad/s (shm v2+)
        "tau_J": state.tau_J.tolist(),  # measured link-side torque Nm, incl. gravity (shm v3+)
        "seq": state.seq,
    }


# ---------------------------------------------------------------------------
# Daemon
# ---------------------------------------------------------------------------
class FrankaTwinDaemon:
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

        self.controller = LocalController(cfg, autostart=True, verbose=verbose)
        # We must serialize ops since move_to* restarts the C++ child and we
        # cannot let another command land between stop/start.
        self._op_lock = threading.Lock()
        self._pub_thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None
        # Gripper commands run on their own thread (see _gripper_start).
        self._gripper_thread: Optional[threading.Thread] = None
        self._gripper_lock = threading.Lock()
        self._gripper_seq = 0
        self._gripper_last: Dict[str, Any] = {}

    # ---------------------------------------------------------------- run loop
    def run(self) -> None:
        self._pub_thread = threading.Thread(
            target=self._state_pub_loop, name="panda_state_pub", daemon=True
        )
        self._pub_thread.start()

        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="panda_watchdog", daemon=True
        )
        self._watchdog_thread.start()

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

    # ---------------------------------------------------------------- watchdog
    def _watchdog_loop(self) -> None:
        """Auto-restart osc_shm if it dies unexpectedly.

        Runs under _op_lock so it can never fire during a move_to* (which
        deliberately stops/restarts the controller and holds the sole FCI
        session). A restart re-seeds the anchor pose; the client's next
        set_ee_target then resumes control.
        """
        while not self._stop_evt.wait(WATCHDOG_POLL_S):
            with self._op_lock:
                if self._stop_evt.is_set():
                    break
                try:
                    if self.controller.ensure_running():
                        logger.info("watchdog: osc_shm restarted")
                except Exception:
                    logger.exception(
                        "watchdog: osc_shm restart failed; retrying in %.1fs",
                        WATCHDOG_POLL_S,
                    )

    def _shutdown(self) -> None:
        logger.info("daemon shutting down")
        if self._gripper_thread is not None and self._gripper_thread.is_alive():
            try:
                self.controller.gripper_stop()
            except Exception:
                logger.exception("gripper_stop at shutdown failed")
            self._gripper_thread.join(timeout=2.0)
        if self._watchdog_thread is not None:
            # Let any in-flight restart settle before we tear down the shm.
            self._watchdog_thread.join(timeout=2.0)
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
        v = req.get("q_max_speed")
        self.controller.move_to_q(q, q_max_speed=v)

    def _op_move_to_pose(self, req: Dict[str, Any]) -> None:
        pos = np.asarray(req["pos"], dtype=np.float64)
        quat = np.asarray(req["quat"], dtype=np.float64)
        v = req.get("q_max_speed")
        self.controller.move_to_pose(pos, quat, q_max_speed=v)

    # -- gripper ------------------------------------------------------------
    # The Franka Hand has its own connection (port 1338), so these never touch
    # osc_shm or the shm segment. They are still blocking on the hardware side
    # (grasp/move < 2 s, homing ~6 s) and the REP loop is serial: a handler that
    # waited would stall every set_ee_target from a policy loop. So homing /
    # move / grasp start a thread and return {"started": true, "seq": n}; the
    # client polls gripper_state until busy == false and last.seq == n.
    def _gripper_busy(self) -> bool:
        t = self._gripper_thread
        return t is not None and t.is_alive()

    def _gripper_start(self, name: str, fn) -> Dict[str, Any]:
        if self._gripper_busy():
            raise RuntimeError("gripper busy: previous command still running (gripper_stop to abort)")
        self._gripper_seq += 1
        seq = self._gripper_seq

        def run() -> None:
            try:
                res = dict(fn())
                res.setdefault("ok", True)
            except Exception as e:  # reported to the client via gripper_state.last
                logger.exception("gripper %s failed", name)
                res = {"ok": False, "cmd": name, "error": str(e)}
            res["seq"] = seq
            with self._gripper_lock:
                self._gripper_last = res

        t = threading.Thread(target=run, name="frankatwin_gripper", daemon=True)
        self._gripper_thread = t
        t.start()
        return {"started": True, "seq": seq}

    def _op_gripper_homing(self, _req: Dict[str, Any]) -> Dict[str, Any]:
        return self._gripper_start("homing", self.controller.gripper_homing)

    def _op_gripper_move(self, req: Dict[str, Any]) -> Dict[str, Any]:
        width = float(req["width"])
        speed = req.get("speed")
        return self._gripper_start("move", lambda: self.controller.gripper_move(width, speed=speed))

    def _op_gripper_grasp(self, req: Dict[str, Any]) -> Dict[str, Any]:
        kw = {k: req.get(k) for k in ("width", "speed", "force", "epsilon_inner", "epsilon_outer")}
        return self._gripper_start("grasp", lambda: self.controller.gripper_grasp(**kw))

    def _op_gripper_stop(self, _req: Dict[str, Any]) -> Dict[str, Any]:
        # Synchronous and quick: signals the running gripper_cmd (or sends a
        # standalone stop). The running command's thread records its final output.
        res = self.controller.gripper_stop()
        if self._gripper_thread is not None:
            self._gripper_thread.join(timeout=2.0)
        return {"result": bool(res.get("result", True))}

    def _op_gripper_state(self, req: Dict[str, Any]) -> Dict[str, Any]:
        """`busy`, `last` (output of the last finished command, incl. its final
        `state`) and -- when `fresh` is true and nothing is running -- a live
        readOnce() as `state`. Live reads cost a TCP round trip to the hand
        (~0.1-0.3 s) during which the REP loop is held, so poll with fresh=false."""
        with self._gripper_lock:
            last = dict(self._gripper_last)
        out: Dict[str, Any] = {"busy": self._gripper_busy(), "last": last}
        if req.get("fresh") and not out["busy"]:
            out["state"] = self.controller.gripper_state()
        return out

    def _op_shutdown(self, _req: Dict[str, Any]) -> Dict[str, Any]:
        # Signal the main loop to stop after we ack the request.
        threading.Thread(target=self.request_stop, daemon=True).start()
        return {"shutting_down": True}


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(prog="frankatwin.daemon")
    parser.add_argument(
        "--config", "-c", type=str, default=None, help="path to robot.yaml"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="verbose logging"
    )
    # End-effector payload overrides (forwarded to osc_shm setLoad at startup).
    # Default: whatever robot.yaml says -- which is mass 0 = bare arm, no
    # setLoad call, Desk-configured load untouched. Pass these when the robot
    # carries something extra, e.g. for the 0.68 kg grasped-object runs:
    #   --load-mass 0.83 --load-com 0 0 0.152
    parser.add_argument(
        "--load-mass", type=float, default=None,
        help="payload mass [kg]; 0 disables setLoad (default: robot.yaml value)",
    )
    parser.add_argument(
        "--load-com", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
        help="flange->payload COM [m] (default: robot.yaml value)",
    )
    parser.add_argument(
        "--load-inertia", type=float, nargs=9, default=None,
        help="payload inertia about COM, row-major 3x3 [kg m^2] "
             "(default: robot.yaml value; auto small diagonal if zero)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cfg = load_config(args.config)
    logger.info("loaded config from %s", cfg.source_path)
    if args.load_mass is not None:
        cfg.load.mass = float(args.load_mass)
    if args.load_com is not None:
        cfg.load.com = [float(v) for v in args.load_com]
    if args.load_inertia is not None:
        cfg.load.inertia = [float(v) for v in args.load_inertia]
    if cfg.load.mass > 0.0 and not any(cfg.load.inertia):
        # libfranka rejects setLoad with an all-zero inertia tensor; the exact
        # value barely matters (gravity comp uses mass + com only).
        cfg.load.inertia = [1.0e-3, 0.0, 0.0, 0.0, 1.0e-3, 0.0, 0.0, 0.0, 1.0e-3]
    if cfg.load.mass > 0.0:
        logger.info(
            "payload: mass=%.3f kg, com=%s m (setLoad at osc_shm startup)",
            cfg.load.mass, cfg.load.com,
        )
    else:
        logger.info("payload: none (bare arm, setLoad skipped)")
    daemon = FrankaTwinDaemon(cfg, verbose=args.verbose)

    def _on_signal(_signo, _frame):
        logger.info("received signal, stopping")
        daemon.request_stop()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    daemon.run()


if __name__ == "__main__":
    main()
