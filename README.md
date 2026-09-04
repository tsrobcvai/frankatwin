<h1 align="center">FrankaTwin</h1>

<p align="center">
A 1 kHz task impedance controller for the <b>Franka Research 3 / Panda</b> whose
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

- **Task impedance control, identical in sim and on the robot.** The controller
  is the task-space impedance law that sim-to-real work such as
  [IndustReal](https://arxiv.org/abs/2305.17110) and
  [OmniReset](https://github.com/uw-lab/omnireset) trains policies on: a
  Cartesian PD wrench mapped through Jᵀ, no null-space term, no apparent-mass
  projection. `src/osc_shm.cpp` (libfranka, 1 kHz) and the IsaacLab controller
  in `isaaclab_sysid/` are the same law with the same gains, damping rule and
  torque slew limit, so a policy runs on the arm through the controller it was
  trained with.
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

Three machines can be involved. Every command below is tagged with where it runs:

| tag | machine | runs | needs |
|---|---|---|---|
| <kbd>NUC</kbd> | real-time PC wired to the robot (FCI) | `python -m frankatwin.daemon` → `osc_shm` / `move_to` | RT kernel, libfranka, C++ build |
| <kbd>PC</kbd> | your workstation | your code, `examples/move_to.py` / `cart_impedance.py`, analysis scripts | Python only |
| <kbd>SIM</kbd> | any GPU box with IsaacLab (can be the PC) | sysid fit, sim replay | IsaacLab ≥ 2.3 |

Each step links to the full page in [docs/](docs/).

### 1. Installation

Full prerequisites (RT kernel, FCI, libfranka ≥ 0.14 + Pinocchio, conda caveats):
[docs/installation.md](docs/installation.md).

<kbd>NUC</kbd> build the 1 kHz controller, install the Python side, start the daemon

```bash
git clone https://github.com/tsrobcvai/frankatwin && cd frankatwin
cmake -S . -B build && cmake --build build -j      # libfranka + Eigen3 (+ Pinocchio for libfranka >= 0.14)
pip install -e .
python -m frankatwin.doctor        # RT kernel, rtprio, binaries, libfranka/pinocchio, FCI link, other FCI clients
python -m frankatwin.daemon        # binds 5555 (commands) / 5556 (state), launches osc_shm
```

<kbd>PC</kbd> Python only

```bash
git clone https://github.com/tsrobcvai/frankatwin && cd frankatwin
pip install -e ".[analysis]"          # analysis: pandas + matplotlib for the compare/plot scripts
vim config/robot.yaml                 # network.nuc_host = the NUC's address as seen from here
python -m frankatwin.doctor                     # daemon reachable? state stream flowing?
```

<kbd>SIM</kbd> only if you will run the sysid loop — see [step 3](#3-system-identification).

`config/robot.yaml` is shared by all sides (network, robot IP, gains, safety
clamps, collision thresholds, payload). Override with `--config` or
`$FRANKATWIN_CONFIG`.

### 2. Basic control

Two controllers exist, and every script below uses one of them:

- **`move_to`** — one-shot **position control** to a joint configuration or an
  EE pose. The trajectory is generated by libfranka (min-jerk in joint space,
  5th-order Cartesian profile for EE targets). Used for resets.
- **`osc_shm`** — **continuous control**: a 1 kHz task impedance controller
  that tracks whatever EE target you stream to it. Used for policies, teleop,
  sysid.

With the daemon running on the NUC, everything in this step is <kbd>PC</kbd>.

| I want to… | run | controller | script |
|---|---|---|---|
| reset to the home pose | `python examples/move_to.py` | position, joint-space | [`examples/move_to.py`](examples/move_to.py) → `move_to_q(robot.init_q)` |
| move to a joint configuration | `python examples/move_to.py --target-joints 0 -0.785 0 -2.356 0 1.571 0.785 [--speed 0.2]` | position, joint-space | same → `move_to_q` |
| move to an EE pose | `python examples/move_to.py --target-ee 0.4 0.0 0.3  0 1 0 0 [--duration 5]` | position, Cartesian | same → `move_to_pose` |
| hold a pose compliantly and nudge it | `python examples/lift_ee.py --height 0.02 --duration 3` | impedance | [`examples/lift_ee.py`](examples/lift_ee.py) — 70 lines, the minimal `set_ee_target` loop |
| track a scripted EE reference + log it | `python examples/cart_impedance.py --kp-pos 500 --kp-ori 30 --log run.csv` | impedance | [`examples/cart_impedance.py`](examples/cart_impedance.py) — `--mode sine` (default, ±5 cm z at 0.5 Hz), `multiband`, `chirp`; prints tracking RMS + torque headroom |
| run a policy closed-loop | `python examples/policy_loop.py --hz 10` | impedance | [`examples/policy_loop.py`](examples/policy_loop.py) — the rollout pattern below, with a stand-in policy |

`--target-ee` format: `x y z` in metres in the robot **base frame** (libfranka's
`O` frame, +x forward, +z up), then the EE orientation as a **unit quaternion in
wxyz order** — the same convention as `RobotState.ee_quat` / `set_ee_target`.
`0 1 0 0` is 180° about x, i.e. tool pointing straight down (the Franka "ready"
orientation). The EE frame is the one configured in Desk (Franka Hand: the TCP
between the fingertips). Home = `robot.init_q` in `config/robot.yaml`.

<kbd>PC</kbd> **a 10 Hz policy on task impedance control** — this is what every closed-loop rollout does

```python
import time, numpy as np
from frankatwin import FrankaTwinClient, load_config
from frankatwin.quat import from_rotvec_wxyz, mul_wxyz

HZ, POS_SCALE, ROT_SCALE = 10, 0.02, 0.05        # policy rate; max |Δpos| [m] and |Δrot| [rad] per step

def policy(obs):                                  # your network; 6-D action in [-1, 1]: Δxyz, Δrot (axis-angle)
    return np.zeros(6)

cfg = load_config()
with FrankaTwinClient(cfg) as robot:
    robot.move_to_q(cfg.robot.init_q)                       # 1. position control: blocking reset to home
    robot.set_gains(kp_pos=500, kp_ori=30,                  # 2. impedance gains (Kd = 2*sqrt(Kp)); clamp 0.15 m
                    error_delta_pos=0.15, error_delta_rot=0.80)   #    > one step, so it never engages (sim has no clip)
    s = robot.wait_for_state()
    t_next = time.monotonic()
    for step in range(300):                                 # 3. 30 s at 10 Hz
        obs = np.concatenate([s.q, s.dq, s.ee_pos, s.ee_quat, s.ee_linvel, s.ee_angvel])
        a = np.clip(policy(obs), -1, 1)
        pos  = s.ee_pos + POS_SCALE * a[:3]                 #    Δ on the measured pose, base frame (as in the sim task)
        quat = mul_wxyz(from_rotvec_wxyz(ROT_SCALE * a[3:]), s.ee_quat)   # world-frame Δrot ⊗ current
        robot.set_ee_target(pos, quat)                      # 4. non-blocking; osc_shm holds it at 1 kHz until next step
        t_next += 1 / HZ
        time.sleep(max(0.0, t_next - time.monotonic()))     # 5. fixed-rate tick, no drift
        s = robot.get_state()                               # 6. newest frame of the 100 Hz stream (<= 10 ms old)
```

What is happening underneath: the policy writes a new pose target every 100 ms;
`osc_shm` reads it on its next 1 ms tick and applies `τ = Jᵀ[Kp e − Kd ẋ]` a
hundred times before the next target arrives (zero-order hold). The IsaacLab
replay reproduces exactly that staircase, which is why the sysid transfers.
`error_delta_pos` is set above the largest single step so the controller never
clips — matching the sim's unclipped task impedance. With `POS_SCALE = 0.02` and
`kp_pos = 500` one step commands at most 10 N.

Rules of thumb: targets are absolute poses in the base frame, quaternions
**wxyz**; a big jump is a torque step (the slew limiter keeps it from tripping a
reflex, but scale your actions); gains and clamps persist across `move_to_*`
and controller restarts. Control law, safety chain and timing:
[docs/architecture.md](docs/architecture.md); every method and config key:
[docs/usage.md](docs/usage.md).

### 3. System identification

From a real excitation run to a PhysX arm that tracks it, in four commands.
Excitation design, parameter bounds and the loss: [docs/sysid.md](docs/sysid.md).

<kbd>PC</kbd> excite the real arm with a 6-DOF chirp, log at 50 Hz

```bash
python examples/cart_impedance.py --mode chirp --kp-pos 500 --kp-ori 30 \
    --err-delta-pos 0.15 --err-delta-rot 0.80 --log data/chirp_$(date +%Y%m%d_%H%M%S).csv
```

<kbd>SIM</kbd> deploy the IsaacLab extension once, fit 29 parameters with CMA-ES, replay with them

```bash
./isaaclab_sysid/install_into_isaaclab.sh /path/to/IsaacLab && pip install cmaes   # once
cd /path/to/IsaacLab
python scripts/tools/sysid_franka_osc.py --headless --num_envs 128 --max_iter 40 \
    --real_csv /path/to/chirp.csv --real_sidecar /path/to/chirp.json
python scripts/tools/apply_sysid_params.py --best logs/sysid_franka/<ts>/sysid_best_params.json \
    --invoke-replay --real-csv /path/to/chirp.csv --real-sidecar /path/to/chirp.json   # -> chirp_sim_sysid.csv
```

<kbd>PC</kbd> overlay real vs sim

```bash
python scripts/compare_sim_real.py --real-csv chirp.csv --sim-csv chirp_sim_sysid.csv --save
```

The fitted `sysid_best_params.json` drops into any IsaacLab Franka task via
`apply_sysid_params.py --print-snippet` (actuator armature / friction / delay
overrides). Results on our arm are in [Results](#results).

### 4. Interfaces

Everything you can talk to, from highest to lowest level:

| interface | where | what | reference |
|---|---|---|---|
| **Scripts** | <kbd>NUC</kbd> `python -m frankatwin.daemon`, `python -m frankatwin.doctor` · <kbd>PC</kbd> `examples/move_to.py`, `examples/lift_ee.py`, `examples/cart_impedance.py`, `scripts/gen_*_traj.py`, `scripts/compare_sim_real.py`, `scripts/check_torque_limits.py`, `python -m frankatwin.doctor` | plain Python files, one job each; read them | [usage.md → Scripts](docs/usage.md#scripts) |
| **Python client** `FrankaTwinClient` | <kbd>PC</kbd> | `set_ee_target(pos, quat_wxyz)`, `set_gains(kp_pos, kp_ori, kd_pos, kd_ori, error_delta_pos, error_delta_rot)`, `enable()` / `disable()`, `get_state(fresh=False)`, `get_state_history()`, `wait_for_state()`, `move_to_q(q, speed_factor)`, `move_to_pose(pos, quat, duration)` | [usage.md → Client API](docs/usage.md#client-api-pc) |
| **`LocalController`** | <kbd>NUC</kbd> | same methods as the client, in-process (no ZMQ) | [usage.md → Client API](docs/usage.md#client-api-pc) |
| **`RobotState`** | both | `timestamp_s, q[7], dq[7], ee_pos[3], ee_quat[4] (wxyz), ee_linvel[3], ee_angvel[3], tau[7]` (commanded), `tau_J[7]` (measured, gravity incl.), `seq` | [usage.md](docs/usage.md#client-api-pc) |
| **Daemon protocol** (any language) | <kbd>PC</kbd> → <kbd>NUC</kbd> | JSON over ZMQ. REQ/REP on `cmd_port`: `{"op": "ping" \| "set_ee_target" \| "set_gains" \| "enable" \| "disable" \| "get_state" \| "move_to_q" \| "move_to_pose" \| "shutdown", ...}` → `{"ok": true, ...}`; PUB on `state_port`: one `RobotState` JSON at 100 Hz | [architecture.md → Daemon](docs/architecture.md#daemon-behaviour) |
| **Shared memory** (any language) | <kbd>NUC</kbd> | `/frankatwin_osc`: `ShmCommand` (seqlock: target, gains, clamps, enabled) and a 1024-frame `ShmStateFrame` ring at 1 kHz. Fixed ABI in `src/shm_layout.h` / `frankatwin.shm_layout`, pinned by `tests/test_shm_layout.py` | [architecture.md → Shared memory](docs/architecture.md#shared-memory) |
| **C++ binaries** | <kbd>NUC</kbd> | `osc_shm <ip> [--shm-name] [--max-torque-rate] [--load-mass/--load-com/--load-inertia] [--collision-torque/--collision-cartesian] [--no-coriolis] [--duration]`; `move_to <ip> --q q1..q7 [--speed-factor]` or `--pose x y z qw qx qy qz [--duration]`; `read_current_q`, `read_current_pose`, `read_load` | `src/*.cpp` headers |
| **Config** `config/robot.yaml` | all | `network`, `robot`, `control` (gains, clamps), `collision`, `paths`, `reset`, `load` | [usage.md → Configuration reference](docs/usage.md#configuration-reference-configrobotyaml) |
| **Data files** | <kbd>PC</kbd> / <kbd>SIM</kbd> | run CSV + sidecar JSON, sim replay CSV, `sysid_best_params.json` | [docs/data_format.md](docs/data_format.md) |
| **IsaacLab tasks** | <kbd>SIM</kbd> | `Isaac-FrankaTwin-Replay-v0`, `Isaac-FrankaTwin-Sysid-v0` (task-impedance controller mirroring `osc_shm`), `franka_mimic.usd` | [docs/sysid.md](docs/sysid.md) |

Not exposed (yet): gripper control, joint-space impedance, relative-pose
targets, ROS. Contributions welcome — see [CONTRIBUTING.md](CONTRIBUTING.md).

## Results

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
python/frankatwin/   daemon, FrankaTwinClient (PC), LocalController (NUC), config, shm_layout,
                     doctor, excitation/ (multiband + chirp reference math)
config/robot.yaml    network, robot IP, gains, safety clamps, collision thresholds, payload
examples/            move_to (home / joints / EE pose), lift_ee, cart_impedance (sine / multiband / chirp)
scripts/             gen_*_traj (write references), compare_sim_real, check_torque_limits, plot_ee_tracking
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
  title   = {FrankaTwin: a sim-to-real aligned 1 kHz task impedance controller for the Franka Research 3},
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
