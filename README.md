<h1 align="center">FrankaTwin</h1>

<p align="center">
A 1 kHz Cartesian impedance controller for the <b>Franka Research 3 / Panda</b> whose
simulation twin is <i>the same controller</i> — plus the system identification that makes
the twin's dynamics match the real arm to within 1–3 % of joint motion range.
</p>

<p align="center">
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/license-Apache--2.0-blue.svg"></a>
  <img alt="libfranka" src="https://img.shields.io/badge/libfranka-0.9%20%E2%80%93%200.15-informational">
  <img alt="IsaacLab" src="https://img.shields.io/badge/IsaacLab-%E2%89%A5%202.3-76b900">
  <img alt="Python" src="https://img.shields.io/badge/python-%E2%89%A5%203.9-3776ab">
</p>

---

## Why FrankaTwin

Most Franka stacks give you a controller. FrankaTwin gives you a controller **and a
proof that the simulated arm behaves like the real one under it**:

- **One control law, two implementations that are kept identical.**
  `src/osc_shm.cpp` (real, libfranka, 1 kHz) and
  `isaaclab_sysid/.../control.py` (IsaacLab, PhysX, 1 kHz) implement the same
  Jacobian-transpose task impedance — no null-space term, no apparent-mass
  projection, same gains, same damping rule, same torque slew limit.
- **System identification that closes the loop.** A CMA-ES fit of 29 parameters
  (per-joint armature, static / dynamic / viscous friction, motor delay) drives the
  sim replay of real excitation runs. Validated on a held-out chirp:
  **joint-position MSE 4.8 × 10⁻⁴ rad²**, per-joint RMSE 12–30 mrad, i.e. 1.9–8.3 %
  of each joint's motion range.
- **Small and auditable.** ~700 lines of C++ for the controller, ~900 lines of
  Python for the daemon/client, POSIX shared memory + ZMQ in between. No ROS.
- **Battle-tested safety.** Torque slew-rate limiter (kills the
  `controller_torque_discontinuity` reflex at setpoint jumps), controlled stop,
  configurable collision thresholds, payload compensation, automatic controller
  restart — each one traceable to a real incident in
  [docs/troubleshooting.md](docs/troubleshooting.md).

<p align="center">
  <img src="docs/images/v3_sysid_v4chirp_joints.png" width="640" alt="Per-joint q/dq: sim (orange) sits on real (blue) on a held-out 6-DOF chirp">
  <br><sub>Held-out 6-DOF chirp: sim (orange) on real (blue), per joint. Target dashed.</sub>
</p>

## Architecture

```
      PC                                  NUC (RT kernel)                       FR3 / FCI
 ┌───────────┐                    ┌──────────────────────────┐              ┌──────────┐
 │ your code │  ZMQ REQ  5555 ──▶ │ frankatwin.daemon        │              │          │
 │ Franka-   │  ZMQ SUB  5556 ◀── │   ├─ POSIX shm           │◀─ 1 kHz ────▶│  robot   │
 │ TwinClient│   ≤ 50 Hz          │   ├─ osc_shm  (C++ 1kHz) │   libfranka  │          │
 └───────────┘                    │   └─ move_to  (C++ reset)│              └──────────┘
                                  └──────────────────────────┘
```

Two control modes are exposed to the PC:

1. **Task impedance** (`osc_shm`, the one you use for policies / teleop / sysid):

   ```
   F = Kp (x_des − x) − Kd ẋ          Kd = 2√Kp (critical damping) unless overridden
   τ = Jᵀ F + C(q, q̇) q̇                per-joint |τ| ≤ τ_max, |dτ/dt| ≤ 800 N·m/s
   ```

2. **Joint-space reset** (`move_to`, libfranka min-jerk `MotionGenerator`) for
   `move_to_q` / `move_to_pose`. The daemon serialises the two — libfranka allows one
   FCI session at a time.

Read [docs/architecture.md](docs/architecture.md) for the shm layout, the seqlock,
the safety chain and the reasoning behind each default.

## Quick start

Full instructions, including the RT-kernel/FCI prerequisites and the
libfranka ≥ 0.14 / Pinocchio situation: [docs/installation.md](docs/installation.md).

**NUC** (builds the C++ binaries, runs the daemon):

```bash
git clone https://github.com/tsrobcvai/frankatwin && cd frankatwin
cmake -S . -B build && cmake --build build -j       # needs libfranka + Eigen3
pip install -e .
python -m pytest tests/test_shm_layout.py -q        # ABI check: C++ struct == numpy dtype
python -m frankatwin.daemon --config config/robot.yaml
```

**PC** (Python only):

```bash
git clone https://github.com/tsrobcvai/frankatwin && cd frankatwin
pip install -e ".[analysis]"
# edit config/robot.yaml: network.nuc_host
python examples/reset_home.py            # joint-space reset via move_to
python examples/cart_impedance.py        # 4 s z-sine around the current pose
```

**From your own code:**

```python
import numpy as np, time
from frankatwin import FrankaTwinClient, load_config

with FrankaTwinClient(load_config()) as robot:
    s = robot.wait_for_state()                    # q, dq, ee_pos, ee_quat (wxyz), tau, tau_J, ...
    anchor_pos, anchor_quat = s.ee_pos, s.ee_quat
    robot.set_gains(kp_pos=500, kp_ori=30)        # Kd defaults to 2*sqrt(Kp)
    for k in range(100):                          # 2 s, 50 Hz
        robot.set_ee_target(anchor_pos + [0, 0, 0.05 * np.sin(2 * np.pi * k / 50)], anchor_quat)
        time.sleep(0.02)
```

`set_ee_target` is non-blocking: it writes a seqlock-guarded frame to shm and
returns; `osc_shm` picks it up on the next 1 kHz tick. See
[docs/usage.md](docs/usage.md) for the full API and configuration reference.

## System identification

The sysid workflow is four commands ([docs/sysid.md](docs/sysid.md) has the details,
the excitation design and the parameter bounds):

```bash
# 1. Excite the real arm with a 6-DOF chirp and log at 50 Hz (PC)
python examples/cart_impedance.py --mode chirp --kp-pos 500 --kp-ori 30 \
    --err-delta-pos 0.15 --err-delta-rot 0.80 --log data/chirp_$(date +%Y%m%d_%H%M%S).csv

# 2. Deploy the IsaacLab extension once, then fit (IsaacLab env)
./isaaclab_sysid/install_into_isaaclab.sh /path/to/IsaacLab && pip install cmaes
cd /path/to/IsaacLab
python scripts/tools/sysid_franka_osc.py --headless --num_envs 128 --max_iter 40 \
    --real_csv /path/to/chirp.csv --real_sidecar /path/to/chirp.json

# 3. Replay the real run in sim with the fitted parameters
python scripts/tools/apply_sysid_params.py --best logs/sysid_franka/<ts>/sysid_best_params.json \
    --invoke-replay --real-csv /path/to/chirp.csv --real-sidecar /path/to/chirp.json

# 4. Overlay (frankatwin repo)
python scripts/compare_sim_real.py --real-csv chirp.csv --sim-csv chirp_sim_sysid.csv --save
```

### Results

Parameters fitted on three multi-band runs (v3), validated on a **held-out** 6-DOF chirp (v4):

| metric (held-out chirp) | value |
|---|---|
| joint-position MSE | **4.8 × 10⁻⁴ rad²** (0.21× the training-set score) |
| per-joint RMSE | 12–30 mrad = 1.9–8.3 % of each joint's motion range |
| EE position / orientation | sim overlays real; both lag the target identically |

On the training trajectories the fit takes the arm from a **41 mm** EE RMS
(un-identified PhysX defaults) to **8.4 mm**, and joint RMS from 983 mrad to 25.5 mrad.
Adding rotation excitation was the key: base-yaw (j1) and wrist-roll (j5) friction
were unidentifiable from translation-only sweeps.

<p align="center">
  <img src="docs/images/v3_sysid_v4chirp_position.png" width="420" alt="EE position: target vs real vs sim">
  <img src="docs/images/v3_sysid_v4chirp_orientation.png" width="420" alt="EE quaternion: target vs real vs sim">
</p>

Fitted parameters (`motor_delay_steps = 1`, i.e. one 1 ms tick):

| joint | armature [kg·m²] | μ_static [N·m] | μ_dynamic [N·m] | μ_viscous [N·m·s/rad] |
|---|---:|---:|---:|---:|
| j1 | 0.382 | 0.73 | 0.29 | 3.68 |
| j2 | 0.159 | 1.17 | 0.90 | 2.29 |
| j3 | 0.157 | 0.59 | 0.34 | 2.87 |
| j4 | 0.174 | 1.03 | 0.81 | 2.37 |
| j5 | 0.239 | 1.63 | 0.92 | 3.30 |
| j6 | 0.180 | 1.14 | 0.58 | 0.79 |
| j7 | 0.060 | 1.06 | 0.50 | 1.94 |

These are for *our* FR3 with a Franka Hand. Friction varies unit to unit; run the fit
on yours — it takes ~4 h on one GPU with 128 parallel envs.

## Repository layout

```
src/                 C++: osc_shm (1 kHz controller), move_to (reset), read_* utilities, shm_layout.h
python/frankatwin/   daemon, FrankaTwinClient (PC), LocalController (NUC), config, shm_layout
config/robot.yaml    network, robot IP, gains, safety clamps, collision thresholds, payload
examples/            cart_impedance (sine / multiband / chirp), reset_home, lift_ee, move_to_q
scripts/             excitation generators, sim-vs-real compare, torque-limit check, plots
isaaclab_sysid/      self-contained IsaacLab extension: tasks, robot USD, sysid/replay scripts
tests/               shm ABI pinning test (C++ offsets vs numpy dtype)
docs/                installation · architecture · usage · sysid · data_format · troubleshooting
```

## Compatibility

| component | tested |
|---|---|
| Robot | Franka Research 3 (system ≥ 5.7) and Panda; Franka Hand attached |
| libfranka | 0.9.x (Panda), 0.13–0.15 (FR3); ≥ 0.14 needs Pinocchio, handled by CMake |
| OS | Ubuntu 20.04 / 22.04 with `PREEMPT_RT` kernel on the NUC |
| IsaacLab | 2.3.0 (needs ≥ 2.3 for the dynamic/viscous joint-friction API) |
| Python | ≥ 3.9 on the PC; ≥ 3.9 on the NUC |

## Safety

FrankaTwin commands torques. Read [docs/troubleshooting.md](docs/troubleshooting.md)
and the *Safety chain* section of [docs/architecture.md](docs/architecture.md) before
running on hardware. Keep the user stop within reach; confirm FCI mode; never run
another FCI client (`franka-interface`, `franka_ros`) at the same time.

## Citing

```bibtex
@software{frankatwin2026,
  author  = {Sun, Tao and Yin, Patrick},
  title   = {FrankaTwin: a sim-to-real aligned 1 kHz Cartesian impedance controller for the Franka Research 3},
  year    = {2026},
  version = {0.2.0},
  url     = {https://github.com/tsrobcvai/frankatwin}
}
```

## Authors

- **Tao Sun** — McGill University
- **Patrick Yin** — University of Washington

## License

Apache-2.0. Vendored code from libfranka (Apache-2.0) and Isaac Lab (BSD-3-Clause)
is listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
