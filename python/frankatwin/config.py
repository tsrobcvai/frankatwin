"""Load and validate frankatwin runtime config.

Config resolution order (first hit wins):
  1. Explicit `path=` argument to `load_config(path=...)` / `--config`.
  2. `$FRANKATWIN_CONFIG`.
  3. `<repo>/config/robot.yaml` -- the checkout this package was installed
     from (`pip install -e .`).

Relative `paths.build_dir` is resolved against the repo root.
"""

from __future__ import annotations

import math
import os
import pathlib
from dataclasses import dataclass, field
from typing import List, Optional

import yaml

_THIS_FILE = pathlib.Path(__file__).resolve()
# python/frankatwin/config.py -> repo root (editable install of a checkout).
REPO_ROOT = _THIS_FILE.parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "robot.yaml"


def resolve_config_path(path: Optional[os.PathLike] = None) -> pathlib.Path:
    """Pick the config file per the module docstring (existence is checked by
    `load_config`)."""
    if path is not None:
        return pathlib.Path(path).expanduser().resolve()
    env = os.environ.get("FRANKATWIN_CONFIG")
    if env:
        return pathlib.Path(env).expanduser().resolve()
    return DEFAULT_CONFIG_PATH


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

    @property
    def kd_pos_effective(self) -> float:
        return self.kd_pos if self.kd_pos is not None else 2.0 * math.sqrt(self.kp_pos)

    @property
    def kd_ori_effective(self) -> float:
        return self.kd_ori if self.kd_ori is not None else 2.0 * math.sqrt(self.kp_ori)


@dataclass
class PathsConfig:
    build_dir: pathlib.Path   # absolute after load (relative -> repo root)
    shm_name: str


@dataclass
class ResetConfig:
    # Single pacing knob for both move_to modes: per-joint velocity cap
    # [rad/s]. See src/move_to.cpp for how each mode consumes it.
    q_max_speed: float = 0.5


@dataclass
class GripperConfig:
    """Franka Hand defaults for the `gripper_*` commands (libfranka `Gripper`).

    Closing is a libfranka *grasp*: the fingers drive towards `grasp_width` and
    squeeze with `grasp_force` once they stall on the object, so the OBJECT sets
    the resting width and `grasp_force` sets how hard it is held. The default
    `grasp_width = -0.01` (past full closure) means "close as far as you can and
    hold". `epsilon_inner/outer` is the band around `grasp_width` inside which
    libfranka reports `is_grasped`; 0.08 (the whole stroke) counts every stall
    as a grasp, matching the deoxys stack this replaces.
    """

    enabled: bool = True
    move_speed: float = 0.1       # m/s, `move` (open)
    grasp_speed: float = 0.5      # m/s, `grasp` (close)
    grasp_force: float = 70.0     # N, Franka Hand rated continuous maximum
    grasp_width: float = -0.01    # m, target the fingers drive towards when closing
    epsilon_inner: float = 0.08   # m
    epsilon_outer: float = 0.08   # m
    max_width: float = 0.08       # m, full stroke of the Franka Hand (`open()` default)


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
class CollisionConfig:
    """libfranka collision-reflex thresholds passed to setCollisionBehavior.

    Each scalar fills ALL entries of the torque (7) / Cartesian (6) lower &
    upper, nominal & acceleration arrays. The task-impedance controller's own
    push is bounded by kp*error_delta (~25 N / ~9 Nm at kp_pos=500/kp_ori=30),
    so thresholds below that range trip "cartesian_reflex" on insertion contact.
    Defaults match the deoxys stack (100), which never reflexed during insertion.
    """

    torque_threshold: float = 100.0      # Nm, per joint
    cartesian_threshold: float = 100.0   # N / Nm, per Cartesian axis


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
    collision: CollisionConfig = field(default_factory=CollisionConfig)
    source_path: Optional[pathlib.Path] = None


def _resolve_path(value: str, base: pathlib.Path) -> pathlib.Path:
    p = pathlib.Path(value).expanduser()
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

    chosen = resolve_config_path(path)
    if not chosen.is_file():
        raise FileNotFoundError(
            f"frankatwin config not found: {chosen} "
            "(pass --config or set $FRANKATWIN_CONFIG)"
        )

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
        )

        p = raw.get("paths", {}) or {}
        paths_cfg = PathsConfig(
            build_dir=_resolve_path(str(p.get("build_dir", "build")), REPO_ROOT),
            shm_name=str(p.get("shm_name", "/frankatwin_osc")),
        )
        if not paths_cfg.shm_name.startswith("/"):
            raise ValueError(
                f"paths.shm_name must start with '/' (got {paths_cfg.shm_name!r})"
            )

        reset = raw.get("reset", {}) or {}
        reset_cfg = ResetConfig(
            q_max_speed=float(reset.get("q_max_speed", 0.5)),
        )
        if not (0.0 < reset_cfg.q_max_speed <= 1.25):
            raise ValueError(
                f"reset.q_max_speed must be in (0, 1.25] rad/s, got "
                f"{reset_cfg.q_max_speed}"
            )

        grip = raw.get("gripper", {}) or {}
        grip_cfg = GripperConfig(
            enabled=bool(grip.get("enabled", True)),
            move_speed=float(grip.get("move_speed", 0.1)),
            grasp_speed=float(grip.get("grasp_speed", 0.5)),
            grasp_force=float(grip.get("grasp_force", 70.0)),
            grasp_width=float(grip.get("grasp_width", -0.01)),
            epsilon_inner=float(grip.get("epsilon_inner", 0.08)),
            epsilon_outer=float(grip.get("epsilon_outer", 0.08)),
            max_width=float(grip.get("max_width", 0.08)),
        )
        for fld in ("move_speed", "grasp_speed", "max_width"):
            if getattr(grip_cfg, fld) <= 0.0:
                raise ValueError(f"gripper.{fld} must be > 0, got {getattr(grip_cfg, fld)}")
        if not (0.0 < grip_cfg.grasp_force <= 70.0):
            raise ValueError(
                f"gripper.grasp_force must be in (0, 70] N (Franka Hand continuous limit), "
                f"got {grip_cfg.grasp_force}"
            )
        if grip_cfg.grasp_width > grip_cfg.max_width:
            raise ValueError(
                f"gripper.grasp_width must be <= max_width ({grip_cfg.max_width}), got {grip_cfg.grasp_width}"
            )
        for fld in ("epsilon_inner", "epsilon_outer"):
            if getattr(grip_cfg, fld) < 0.0:
                raise ValueError(f"gripper.{fld} must be >= 0, got {getattr(grip_cfg, fld)}")

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

        coll = raw.get("collision", {}) or {}
        collision_cfg = CollisionConfig(
            torque_threshold=float(coll.get("torque_threshold", 100.0)),
            cartesian_threshold=float(coll.get("cartesian_threshold", 100.0)),
        )
        for fld in ("torque_threshold", "cartesian_threshold"):
            if getattr(collision_cfg, fld) <= 0.0:
                raise ValueError(f"collision.{fld} must be > 0, got {getattr(collision_cfg, fld)}")

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
        collision=collision_cfg,
        source_path=chosen,
    )
