# Third-party notices

FrankaTwin is licensed under Apache-2.0 (see [LICENSE](LICENSE)). It vendors or
derives from the following third-party code:

| Path | Origin | License |
|---|---|---|
| `src/examples_common.{h,cpp}` | [libfranka](https://github.com/frankaemika/libfranka) `examples/examples_common.*` — `MotionGenerator` and `setDefaultBehavior`, vendored verbatim | Apache-2.0, © Franka Robotics GmbH |
| `isaaclab_sysid/source/isaaclab_tasks/.../franka_sysid/` | Derived from the [Isaac Lab](https://github.com/isaac-sim/IsaacLab) `DirectRLEnv` task template and its OSC controller utilities | BSD-3-Clause, © Isaac Lab Project Developers |
| `isaaclab_sysid/scripts/tools/{sysid_franka_osc,replay_python_csv_sim,apply_sysid_params}.py` | Written against the Isaac Lab script conventions (`AppLauncher`, `parse_env_cfg`); carry the Isaac Lab header. `compare_sim_real.py` in the same directory is FrankaTwin's own (numpy / pandas / matplotlib only) and is Apache-2.0 like the rest of the repository | BSD-3-Clause, © Isaac Lab Project Developers |
| `isaaclab_sysid/source/isaaclab_assets/data/Robots/Franka/franka_mimic.usd` | NVIDIA's Franka Emika Panda `franka_mimic.usd`, the asset Isaac Lab's Forge / Factory tasks use (the file's own metadata records it as generated from `isaaclab_assets/data/Robots/Forge/franka_mimic.usd`); redistributed so the tasks run from a plain Isaac Lab checkout, which does not ship it | NVIDIA Omniverse asset license |
| `python/frankatwin/excitation/chirp.py` (design only) | The 6-DOF linear-chirp excitation shape follows the UR5e `collect_sysid_data.py` of [OmniReset](https://weirdlabuw.github.io/omnireset/) ([paper](https://arxiv.org/abs/2603.15789)); that script is not published, its command line appears in [UWLab's sim2real notes](https://github.com/UW-Lab/UWLab/blob/main/docs/source/publications/omnireset/sim2real.rst). No code is copied | — |

Documentation images — `docs/images/deployment.svg` embeds cropped, downscaled
photographs from Wikimedia Commons:

| Machine | File (Wikimedia Commons) | Author | License |
|---|---|---|---|
| Robot | [Franka Emika3.jpg](https://commons.wikimedia.org/wiki/File:Franka_Emika3.jpg) | Ims | CC BY-SA 4.0 |
| NUC | [Intel NUC8.jpg](https://commons.wikimedia.org/wiki/File:Intel_NUC8.jpg) | Laserlicht | CC BY-SA 4.0 |
| PC | [Corsair 4000D Airflow mid-tower ATX case.tif](https://commons.wikimedia.org/wiki/File:Corsair_4000D_Airflow_mid-tower_ATX_case.tif) | PJ | CC BY-SA 4.0 |
| SIM | [Nvidia GeForce RTX 5060 Ti 16GB, PNY Overclocked Dual Fan, front.jpg](https://commons.wikimedia.org/wiki/File:Nvidia_GeForce_RTX_5060_Ti_16GB,_PNY_Overclocked_Dual_Fan,_front.jpg) | FreeMediaKid! | CC BY-SA 4.0 |

Runtime dependencies (not redistributed): libfranka (Apache-2.0), Eigen (MPL-2.0),
Pinocchio (BSD-2-Clause), Boost (BSL-1.0), NumPy (BSD-3-Clause), PyYAML (MIT),
pyzmq (BSD-3-Clause), cmaes (MIT), Isaac Lab (BSD-3-Clause), Isaac Sim (NVIDIA EULA).
