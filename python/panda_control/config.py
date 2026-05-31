"""Load and validate panda_control runtime config.

Config resolution order:
  1. Explicit `path=` argument to `load_config(path=...)`.
  2. `PANDA_CONFIG` environment variable.
  3. `<repo_root>/config/robot.yaml`  (repo default).

Repo root is detected by walking upward from this file until a directory
containing both `src/` and `config/` is found.
"""

from __future__ import annotations

import math
import os
import pathlib
from dataclasses import dataclass, field
from typing import List, Optional

import yaml

_THIS_FILE = pathlib.Path(__file__).resolve()


def _detect_repo_root() -> pathlib.Path:
    cur = _THIS_FILE.parent
    for _ in range(6):
        if (cur / "CMakeLists.txt").exists() and (cur / "src").is_dir():
            return cur
        if cur.parent == cur:
            break
        cur = cur.parent
    # Fallback: assume the parent of the python package directory.
    return _THIS_FILE.parents[2]


REPO_ROOT = _detect_repo_root()
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "robot.yaml"


@dataclass
class NetworkConfig:
    nuc_host: str
    cmd_port: int
    state_port: int
    state_cache: int = 256


@dataclass
class RobotSpec:
    ip: str
    init_q: List[float]


@dataclass
class ControlConfig:
    frequency_hz: int
    kp_pos: float
    kp_ori: float
    kd_pos: Optional[float]
    kd_ori: Optional[float]
    error_delta_pos: float
    error_delta_rot: float

    @property
    def kd_pos_effective(self) -> float:
        return self.kd_pos if self.kd_pos is not None else 2.0 * math.sqrt(self.kp_pos)

    @property
    def kd_ori_effective(self) -> float:
        return self.kd_ori if self.kd_ori is not None else 2.0 * math.sqrt(self.kp_ori)


@dataclass
class PathsConfig:
    build_dir: pathlib.Path
    shm_name: str


@dataclass
class ResetConfig:
    joint_speed_factor: float = 0.2
    pose_duration: float = 5.0


@dataclass
class GripperConfig:
    enabled: bool = False


@dataclass
class LoadConfig:
    """End-effector payload (e.g. a mounted camera) for gravity compensation.

    mass <= 0 means "no extra load" -> osc_shm leaves the Desk-configured load
    untouched (calls no setLoad).  com is the flange->load COM vector [m];
    inertia is the 3x3 about the COM, row-major [kg m^2].
    """

    mass: float = 0.0
    com: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    inertia: List[float] = field(default_factory=lambda: [0.0] * 9)


@dataclass
class RobotConfig:
    """Top-level config bundle. Pass-through for clients/daemon/local."""

    network: NetworkConfig
    robot: RobotSpec
    control: ControlConfig
    paths: PathsConfig
    reset: ResetConfig = field(default_factory=ResetConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    load: LoadConfig = field(default_factory=LoadConfig)
    source_path: Optional[pathlib.Path] = None


def _resolve_path(value: str, base: pathlib.Path) -> pathlib.Path:
    p = pathlib.Path(value)
    return p if p.is_absolute() else (base / p).resolve()


def _validate_init_q(q: List[float]) -> List[float]:
    if not isinstance(q, list) or len(q) != 7:
        raise ValueError(f"robot.init_q must be a list of 7 floats, got {q!r}")
    out: List[float] = []
    for i, v in enumerate(q):
        if not isinstance(v, (int, float)):
            raise ValueError(f"robot.init_q[{i}] is not numeric: {v!r}")
        out.append(float(v))
    return out


def _validate_quat(v) -> None:
    if v is None:
        return
    if not isinstance(v, (int, float)) or v < 0:
        raise ValueError(f"expected non-negative number, got {v!r}")


def load_config(path: Optional[os.PathLike] = None) -> RobotConfig:
    """Load and validate robot.yaml. See module docstring for resolution rules."""

    chosen: pathlib.Path
    if path is not None:
        chosen = pathlib.Path(path).expanduser().resolve()
    elif "PANDA_CONFIG" in os.environ:
        chosen = pathlib.Path(os.environ["PANDA_CONFIG"]).expanduser().resolve()
    else:
        chosen = DEFAULT_CONFIG_PATH

    if not chosen.is_file():
        raise FileNotFoundError(f"panda_control config not found: {chosen}")

    with chosen.open("r") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ValueError(f"config root must be a mapping, got {type(raw).__name__}")

    try:
        net = raw["network"]
        net_cfg = NetworkConfig(
            nuc_host=str(net["nuc_host"]),
            cmd_port=int(net["cmd_port"]),
            state_port=int(net["state_port"]),
            state_cache=int(net.get("state_cache", 256)),
        )
        if net_cfg.cmd_port == net_cfg.state_port:
            raise ValueError("network.cmd_port and network.state_port must differ")

        rob = raw["robot"]
        robot_spec = RobotSpec(
            ip=str(rob["ip"]),
            init_q=_validate_init_q(rob["init_q"]),
        )

        ctrl = raw["control"]
        for key in ("kp_pos", "kp_ori"):
            if not isinstance(ctrl[key], (int, float)) or ctrl[key] <= 0:
                raise ValueError(f"control.{key} must be a positive number")
        _validate_quat(ctrl.get("kd_pos"))
        _validate_quat(ctrl.get("kd_ori"))
        ctrl_cfg = ControlConfig(
            frequency_hz=int(ctrl.get("frequency_hz", 1000)),
            kp_pos=float(ctrl["kp_pos"]),
            kp_ori=float(ctrl["kp_ori"]),
            kd_pos=None if ctrl.get("kd_pos") is None else float(ctrl["kd_pos"]),
            kd_ori=None if ctrl.get("kd_ori") is None else float(ctrl["kd_ori"]),
            error_delta_pos=float(ctrl.get("error_delta_pos", 0.05)),
            error_delta_rot=float(ctrl.get("error_delta_rot", 0.30)),
        )

        p = raw["paths"]
        base = chosen.parent.parent  # config/ -> repo root
        paths_cfg = PathsConfig(
            build_dir=_resolve_path(str(p["build_dir"]), base),
            shm_name=str(p.get("shm_name", "/panda_osc")),
        )
        if not paths_cfg.shm_name.startswith("/"):
            raise ValueError(
                f"paths.shm_name must start with '/' (got {paths_cfg.shm_name!r})"
            )

        reset = raw.get("reset", {}) or {}
        reset_cfg = ResetConfig(
            joint_speed_factor=float(reset.get("joint_speed_factor", 0.2)),
            pose_duration=float(reset.get("pose_duration", 5.0)),
        )
        if not (0.0 < reset_cfg.joint_speed_factor <= 0.5):
            raise ValueError(
                f"reset.joint_speed_factor must be in (0, 0.5], got "
                f"{reset_cfg.joint_speed_factor}"
            )

        grip = raw.get("gripper", {}) or {}
        grip_cfg = GripperConfig(enabled=bool(grip.get("enabled", False)))

        load = raw.get("load", {}) or {}
        load_cfg = LoadConfig(
            mass=float(load.get("mass", 0.0)),
            com=[float(x) for x in load.get("com", [0.0, 0.0, 0.0])],
            inertia=[float(x) for x in load.get("inertia", [0.0] * 9)],
        )
        if load_cfg.mass < 0.0:
            raise ValueError(f"load.mass must be >= 0, got {load_cfg.mass}")
        if len(load_cfg.com) != 3:
            raise ValueError(f"load.com must have 3 elements, got {load_cfg.com!r}")
        if len(load_cfg.inertia) != 9:
            raise ValueError(f"load.inertia must have 9 elements, got {load_cfg.inertia!r}")

    except KeyError as e:
        raise KeyError(f"missing required config key: {e}") from e

    return RobotConfig(
        network=net_cfg,
        robot=robot_spec,
        control=ctrl_cfg,
        paths=paths_cfg,
        reset=reset_cfg,
        gripper=grip_cfg,
        load=load_cfg,
        source_path=chosen,
    )
