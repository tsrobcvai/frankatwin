"""Console entry points: frankatwin-reset and frankatwin-doctor.

(`frankatwin-daemon`, `frankatwin-excite` and `frankatwin-gen-*` live next to
the code they wrap; see ``[project.scripts]`` in pyproject.toml.)
"""

from __future__ import annotations

import argparse
import os
import pathlib
import platform
import shutil
import socket
import subprocess
import sys
from typing import List, Optional, Sequence

from frankatwin import __version__
from frankatwin.config import RobotConfig, load_config, resolve_config_path

FCI_TCP_PORT = 1337  # libfranka command channel


# =============================================================================
# frankatwin-reset
# =============================================================================
def reset_main(argv: Optional[Sequence[str]] = None) -> int:
    """Blocking position-controlled move via ``move_to``.

    Default: joint-space move to ``robot.init_q`` (home). ``--q`` picks another
    joint target; ``--pose`` does an EE-pose move instead (libfranka
    CartesianPose, no IK on our side). Either way osc_shm is stopped for the
    move and restarted anchored at the new pose, gains preserved.
    """
    import numpy as np

    from frankatwin.remote_client import FrankaTwinClient

    p = argparse.ArgumentParser(
        prog="frankatwin-reset",
        description="Position-controlled move (joint-space by default, --pose for an EE pose): "
                    "stops osc_shm, runs move_to, restarts osc_shm at the new pose.",
    )
    p.add_argument("--config", "-c", default=None, help="path to robot.yaml")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--q", type=float, nargs=7, default=None, metavar="Q",
                   help="joint target [rad] x7 (default: robot.init_q)")
    g.add_argument("--pose", type=float, nargs=7, default=None, metavar="P",
                   help="EE target: x y z [m] qw qx qy qz (base frame, wxyz)")
    p.add_argument("--speed", type=float, default=None,
                   help="joint move: MotionGenerator speed factor (0, 0.5] (default: reset.joint_speed_factor)")
    p.add_argument("--duration", type=float, default=None,
                   help="pose move: seconds, [1.5, 20] (default: reset.pose_duration)")
    args = p.parse_args(argv)

    cfg = load_config(args.config)
    with FrankaTwinClient(cfg) as robot:
        if args.pose is not None:
            pos, quat = np.asarray(args.pose[:3], dtype=np.float64), np.asarray(args.pose[3:], dtype=np.float64)
            print(f"[frankatwin-reset] moving EE to pos = {pos.tolist()}, quat(wxyz) = {quat.tolist()}")
            robot.move_to_pose(pos, quat, duration=args.duration)
        else:
            q = np.asarray(args.q if args.q is not None else cfg.robot.init_q, dtype=np.float64)
            print(f"[frankatwin-reset] moving to q = {np.round(q, 4).tolist()}")
            robot.move_to_q(q, speed_factor=args.speed)
        s = robot.wait_for_state(timeout_s=5.0)
    print(f"[frankatwin-reset] done; q = {np.round(s.q, 4).tolist()}")
    print(f"[frankatwin-reset]       ee_pos = {np.round(s.ee_pos, 4).tolist()}, ee_quat(wxyz) = {np.round(s.ee_quat, 4).tolist()}")
    return 0


# =============================================================================
# frankatwin-doctor
# =============================================================================
class _Report:
    OK, WARN, FAIL, SKIP = "ok", "!!", "XX", "--"

    def __init__(self) -> None:
        self.lines: List[tuple] = []
        self.failed = 0

    def add(self, status: str, name: str, detail: str = "", hint: str = "") -> None:
        self.lines.append((status, name, detail, hint))
        if status == self.FAIL:
            self.failed += 1

    def render(self) -> str:
        out = []
        for status, name, detail, hint in self.lines:
            out.append(f"[{status}] {name:<22s} {detail}")
            if hint and status in (self.FAIL, self.WARN):
                out.append(f"     -> {hint}")
        return "\n".join(out)


def _tcp_reachable(host: str, port: int, timeout: float = 1.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _run(cmd: List[str], timeout: float = 5.0) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""


def _check_common(rep: _Report, cfg_path: pathlib.Path, cfg: Optional[RobotConfig], err: Optional[str]) -> None:
    rep.add(_Report.OK, "python", f"{platform.python_version()}  frankatwin {__version__}")
    if cfg is None:
        rep.add(_Report.FAIL, "config", f"{cfg_path}: {err}",
                "fix config/robot.yaml, or pass --config / set $FRANKATWIN_CONFIG")
        return
    rep.add(_Report.OK, "config", str(cfg_path))
    rep.add(_Report.OK, "  network", f"nuc_host={cfg.network.nuc_host} cmd={cfg.network.cmd_port} state={cfg.network.state_port}")
    rep.add(_Report.OK, "  robot", f"ip={cfg.robot.ip}  kp={cfg.control.kp_pos:g}/{cfg.control.kp_ori:g}  "
            f"err_delta={cfg.control.error_delta_pos:g}/{cfg.control.error_delta_rot:g}")


def _check_nuc(rep: _Report, cfg: RobotConfig) -> None:
    # Realtime kernel + rtprio
    rt_flag = pathlib.Path("/sys/kernel/realtime")
    is_rt = (rt_flag.is_file() and rt_flag.read_text().strip() == "1") or "PREEMPT_RT" in platform.version()
    rep.add(_Report.OK if is_rt else _Report.WARN, "rt kernel", platform.release(),
            "install a PREEMPT_RT kernel (libfranka needs it for the 1 kHz loop)")
    try:
        import resource
        soft, _hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
        ok = soft == resource.RLIM_INFINITY or soft >= 80
        rep.add(_Report.OK if ok else _Report.WARN, "rtprio limit", str(soft),
                "add your user to the `realtime` group (limits.conf) or `setcap cap_sys_nice+ep <osc_shm>`")
    except Exception:
        rep.add(_Report.SKIP, "rtprio limit", "n/a")

    # Binaries
    osc: Optional[pathlib.Path] = None
    for name in ("osc_shm", "move_to"):
        path = cfg.paths.build_dir / name
        if path.is_file():
            rep.add(_Report.OK, f"binary {name}", str(path))
            if name == "osc_shm":
                osc = path
        else:
            rep.add(_Report.FAIL, f"binary {name}", f"not found at {path}",
                    "cmake -S . -B build && cmake --build build   (docs/installation.md), "
                    "or set paths.build_dir in robot.yaml")
    if osc is not None:
        ldd = _run(["ldd", str(osc)])
        fr = [l.strip() for l in ldd.splitlines() if "libfranka" in l]
        rep.add(_Report.OK if fr else _Report.WARN, "libfranka", fr[0] if fr else "not in ldd output",
                "osc_shm must link libfranka; rebuild against a libfranka install")
        if fr and "not found" in fr[0]:
            rep.add(_Report.FAIL, "libfranka", fr[0], "libfranka .so not on the loader path (LD_LIBRARY_PATH / ldconfig)")
        pin = [l.strip() for l in ldd.splitlines() if "pinocchio" in l]
        if pin:
            bad = [l for l in pin if "not found" in l]
            rep.add(_Report.FAIL if bad else _Report.OK, "pinocchio", (bad or pin)[0],
                    "pinocchio .so not on the loader path")
        if shutil.which("getcap"):
            caps = _run(["getcap", str(osc)]).strip()
            rep.add(_Report.OK, "capabilities", caps or "none (needs rtprio via realtime group)")

    # FCI
    if _tcp_reachable(cfg.robot.ip, FCI_TCP_PORT):
        rep.add(_Report.OK, "fci", f"{cfg.robot.ip}:{FCI_TCP_PORT} reachable")
    else:
        rep.add(_Report.FAIL, "fci", f"{cfg.robot.ip}:{FCI_TCP_PORT} unreachable",
                "check the FCI link (172.16.0.x), that FCI is enabled in Desk, and robot.ip in robot.yaml")

    # Competing FCI clients
    procs = _run(["pgrep", "-a", "-f", r"franka-interface|franka_control|franka_ros|osc_shm|move_to"])
    me = str(os.getpid())
    others = [l for l in procs.splitlines() if l and not l.startswith(me + " ")]
    if others:
        rep.add(_Report.WARN, "fci clients", "; ".join(o[:60] for o in others[:3]),
                "only one FCI session at a time -- stop them unless it is your own frankatwin-daemon")
    else:
        rep.add(_Report.OK, "fci clients", "none running")

    # shm segment
    seg = pathlib.Path("/dev/shm") / cfg.paths.shm_name.lstrip("/")
    if seg.exists():
        rep.add(_Report.OK, "shm segment", f"{seg} present ({seg.stat().st_size} B)")
    else:
        rep.add(_Report.OK, "shm segment", f"{seg} absent (created by frankatwin-daemon)")


def _check_pc(rep: _Report, cfg: RobotConfig) -> None:
    try:
        import zmq
    except ImportError:
        rep.add(_Report.FAIL, "pyzmq", "not installed", "pip install pyzmq")
        return
    ctx = zmq.Context.instance()
    url = f"tcp://{cfg.network.nuc_host}:{cfg.network.cmd_port}"
    req = ctx.socket(zmq.REQ)
    req.setsockopt(zmq.LINGER, 0)
    req.connect(url)
    try:
        req.send_json({"op": "ping"})
        if req.poll(2000):
            reply = req.recv_json()
            pid = reply.get("pid") if isinstance(reply, dict) else None
            if pid:
                rep.add(_Report.OK, "daemon", f"{url} replies; osc_shm pid {pid}")
            else:
                rep.add(_Report.WARN, "daemon", f"{url} replies but osc_shm pid is 0",
                        "controller not running -- check the daemon log (-v)")
        else:
            rep.add(_Report.FAIL, "daemon", f"no reply from {url} in 2 s",
                    "start `frankatwin-daemon` on the NUC; check network.nuc_host / firewall on the cmd port")
            return
    finally:
        req.close()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect(f"tcp://{cfg.network.nuc_host}:{cfg.network.state_port}")
    try:
        n = 0
        poller = zmq.Poller()
        poller.register(sub, zmq.POLLIN)
        import time
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            if poller.poll(100):
                sub.recv(flags=zmq.NOBLOCK)
                n += 1
        if n:
            rep.add(_Report.OK, "state stream", f"{n} frames / s")
        else:
            rep.add(_Report.FAIL, "state stream", "no frames in 1 s",
                    "firewall on the state port, or osc_shm is not publishing (daemon log)")
    finally:
        sub.close()


def doctor_main(argv: Optional[Sequence[str]] = None) -> int:
    """Check the environment and print a report; exit 1 on any hard failure."""
    p = argparse.ArgumentParser(
        prog="frankatwin-doctor",
        description="Check RT kernel, binaries, libfranka, FCI reachability, daemon and state stream.",
    )
    p.add_argument("--config", "-c", default=None, help="path to robot.yaml")
    p.add_argument("--role", choices=["auto", "nuc", "pc"], default="auto",
                   help="nuc: controller-host checks; pc: client checks; auto: nuc checks if the "
                        "robot's FCI port is reachable or binaries are present, plus pc checks")
    args = p.parse_args(argv)

    rep = _Report()
    cfg_path = resolve_config_path(args.config)
    cfg: Optional[RobotConfig] = None
    err: Optional[str] = None
    try:
        cfg = load_config(args.config)
    except Exception as e:  # report, don't crash
        err = str(e)
    _check_common(rep, cfg_path, cfg, err)
    if cfg is not None:
        role = args.role
        if role == "auto":
            has_bin = (cfg.paths.build_dir / "osc_shm").is_file()
            role = "nuc" if (has_bin or _tcp_reachable(cfg.robot.ip, FCI_TCP_PORT, 0.5)) else "pc"
            rep.add(_Report.OK, "role", f"{role} (auto)")
        if role == "nuc":
            _check_nuc(rep, cfg)
        _check_pc(rep, cfg)
    print(rep.render())
    if rep.failed:
        print(f"\n{rep.failed} problem(s). See docs/troubleshooting.md.")
        return 1
    print("\nall good.")
    return 0


if __name__ == "__main__":  # python -m frankatwin.cli <doctor|config|reset> ...
    _cmd = sys.argv[1] if len(sys.argv) > 1 else "doctor"
    sys.exit({"doctor": doctor_main, "reset": reset_main}[_cmd](sys.argv[2:]))
