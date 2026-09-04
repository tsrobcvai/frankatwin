# Third-party notices

FrankaTwin is licensed under Apache-2.0 (see [LICENSE](LICENSE)). It vendors or
derives from the following third-party code:

| Path | Origin | License |
|---|---|---|
| `src/examples_common.{h,cpp}` | [libfranka](https://github.com/frankaemika/libfranka) `examples/examples_common.*` — `MotionGenerator` and `setDefaultBehavior`, vendored verbatim | Apache-2.0, © Franka Robotics GmbH |
| `isaaclab_sysid/source/isaaclab_tasks/.../franka_sysid/` | Derived from the [Isaac Lab](https://github.com/isaac-sim/IsaacLab) `DirectRLEnv` task template and its OSC controller utilities | BSD-3-Clause, © Isaac Lab Project Developers |
| `isaaclab_sysid/scripts/tools/*.py` | Written against the Isaac Lab script conventions (`AppLauncher`, `parse_env_cfg`); carry the Isaac Lab header | BSD-3-Clause, © Isaac Lab Project Developers |
| `isaaclab_sysid/source/isaaclab_assets/data/Robots/Franka/franka_mimic.usd` | Franka Emika Panda USD from the Isaac Sim asset pack with an added `panda_fingertip_centered` frame | NVIDIA Omniverse asset license |
| `python/frankatwin/excitation/chirp.py` (design only) | The 6-DOF linear-chirp excitation shape follows the UR5e `collect_sysid_data.py` in [uw-lab/omnireset](https://github.com/uw-lab/omnireset); no code is copied | — |

Runtime dependencies (not redistributed): libfranka (Apache-2.0), Eigen (MPL-2.0),
Pinocchio (BSD-2-Clause), Boost (BSL-1.0), NumPy (BSD-3-Clause), PyYAML (MIT),
pyzmq (BSD-3-Clause), cmaes (MIT), Isaac Lab (BSD-3-Clause), Isaac Sim (NVIDIA EULA).
