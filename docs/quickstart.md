# Quick start

## 0. Prerequisites

Three machines can be involved. Every command below is tagged with where it runs:

| tag | machine | runs | software (tested) |
|---|---|---|---|
| — | **Robot** | — | Franka Research 3 (system ≥ 5.7) or Panda, Franka Hand attached, FCI enabled in Desk |
| <kbd>NUC</kbd> | real-time PC wired to the robot (FCI) | `python -m frankatwin.daemon` → `osc_shm` / `move_to` | Ubuntu 20.04 / 22.04 with `PREEMPT_RT` kernel · libfranka 0.13–0.15; ≥ 0.14 needs Pinocchio, handled by CMake · Eigen3, CMake ≥ 3.10 · Python ≥ 3.9 |
| <kbd>PC</kbd> | your workstation | `examples/*.py`, analysis scripts | Python ≥ 3.9 (numpy, pyyaml, pyzmq; pandas + matplotlib for the analysis scripts) |
| <kbd>SIM</kbd> | any GPU box with IsaacLab (can be the PC) | sysid fit, sim replay | IsaacLab 2.3.0 (≥ 2.3 for the dynamic/viscous joint-friction API) · `cmaes` |

Each step links to the full page of this guide.

## 1. Installation

Full prerequisites (RT kernel, FCI, libfranka ≥ 0.14 + Pinocchio, conda caveats):
[docs/installation.md](installation.md).

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

## 2. Basic control

Four steps: bring the controller up on the NUC, then drive the arm from the PC
with three scripts (each takes `--config robot.yaml`; the two controllers they
use are described in [Architecture](architecture.md)).

### Step 1 · Start the daemon

<kbd>NUC</kbd>

```bash
python -m frankatwin.daemon -v
```

It launches `osc_shm` — the arm now holds its current pose under impedance
control — and keeps it alive. Leave the terminal open; the banner should show
`RT = SCHED_FIFO`, `tau_rate = 800 Nm/s`, the payload and collision settings,
then `daemon ready`. All flags (`--config`, payload overrides), the banner line
by line, what it logs while running, how to stop it and when to restart it:
[docs/usage.md → Daemon](usage.md#daemon-nuc).

### Step 2 · Reset the arm

<kbd>PC</kbd> · script: [`examples/move_to.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/move_to.py)

One-shot position control (`move_to`). Home by default; prints the pose
`osc_shm` holds afterwards.

```bash
python examples/move_to.py                                                # home = robot.init_q
python examples/move_to.py --target-joints 0 -0.785 0 -2.356 0 1.571 0.785 --speed 0.2
python examples/move_to.py --target-ee 0.4 0.0 0.3  0 1 0 0 --duration 5
```

| flag | meaning |
|---|---|
| `--target-joints J1 … J7` | 7 absolute joint angles [rad] |
| `--target-ee x y z qw qx qy qz` | EE position [m] in the robot **base frame** (+x forward, +z up) and orientation as a **unit quaternion, wxyz** — `0 1 0 0` = tool pointing down. EE frame = the one configured in Desk (Franka Hand: TCP between the fingertips) |
| `--speed` | joint move: speed factor (0, 0.5]; default `reset.joint_speed_factor` |
| `--duration` | EE move: seconds in [1.5, 20]; default `reset.pose_duration` |

### Step 3 · Track a scripted reference

<kbd>PC</kbd> · script: [`examples/cart_impedance.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/cart_impedance.py)

Continuous control (`osc_shm`) following a scripted EE reference at `--rate` Hz.
Logs a per-tick CSV + sidecar and prints tracking RMS and torque headroom. This
is also the sysid data collector.

```bash
python examples/cart_impedance.py                                         # sine: ±5 cm z at 0.5 Hz for 4 s
python examples/cart_impedance.py --mode chirp --kp-pos 500 --kp-ori 30 \
    --err-delta-pos 0.15 --err-delta-rot 0.80 --log data/run.csv        # sysid v4 chirp, logged
python examples/cart_impedance.py --mode multiband --dry-run              # build + check the reference, no robot
```

| flag | meaning |
|---|---|
| `--mode sine \| multiband \| chirp` | reference: z-sine (default), sysid v3 multi-band, sysid v4 chirp |
| `--kp-pos`, `--kp-ori` | impedance gains; default `control.*` in `robot.yaml` |
| `--err-delta-pos`, `--err-delta-rot` | `osc_shm` error clamps; the chirp needs `0.15` / `0.80` |
| `--rate`, `--duration` | loop rate [Hz] (50) and run time [s] (4 / 12 / 8 by mode) |
| `--amp`, `--freq` · `--amp-x/y/z`, `--amp-yaw`, `--amp-roll` · `--f0`, `--f1`, `--amp-rx/ry/rz` | reference shape for sine · multiband · chirp |
| `--log run.csv [--sidecar run.json]` | write the CSV + metadata ([format](data_format.md)) |
| `--dry-run` | build the reference and print peak rates without a robot |

### Step 4 · Run a policy closed-loop

<kbd>PC</kbd> · script: [`examples/policy_loop.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/policy_loop.py)

Continuous control driven by a policy at a fixed rate. Ships with a stand-in
policy that moves the EE up 10 cm and back down every 4 s for 16 s; swap in
your network.

```bash
python examples/policy_loop.py                          # demo policy, 10 Hz, 16 s
python examples/policy_loop.py --hz 20 --pos-scale 0.0025 --no-reset
```

| flag | meaning |
|---|---|
| `--hz`, `--duration` | policy rate [Hz] (10) and run time [s] (16) |
| `--pos-scale`, `--rot-scale` | action → Δpos [m/step] (0.005) and Δrot [rad/step] (0.02) |
| `--kp-pos`, `--kp-ori`, `--err-delta-pos`, `--err-delta-rot` | impedance gains (500 / 30) and clamps (0.15 / 0.80) |
| `--no-reset` | skip the initial `move_to` home |

The whole loop — the pattern every closed-loop rollout uses:

```python
def demo_policy(t, obs):                      # stand-in for a network; 6-D action in [-1, 1]
    a = np.zeros(6)                           # a[0:3] = Δxyz, a[3:6] = Δrot (axis-angle), base frame
    a[2] = 1.0 if t % 4.0 < 2.0 else -1.0     # up for 2 s, down for 2 s: 10 cm at 0.005 m/step, 10 Hz
    return a

with FrankaTwinClient(cfg) as robot:
    robot.move_to_q(cfg.robot.init_q)                         # 1. position control: reset to home
    robot.set_gains(kp_pos=500, kp_ori=30,                    # 2. impedance gains (Kd = 2*sqrt(Kp));
                    error_delta_pos=0.15, error_delta_rot=0.80)   #    clamp > one step, so it never engages
    s = robot.wait_for_state()
    t0 = time.monotonic()
    for k in range(int(16 * 10)):                             # 3. 16 s at 10 Hz
        obs = np.concatenate([s.q, s.dq, s.ee_pos, s.ee_quat, s.ee_linvel, s.ee_angvel])
        a = np.clip(demo_policy(k / 10, obs), -1, 1)
        pos  = s.ee_pos + 0.005 * a[:3]                       # 4. Δ on the measured pose (as in the sim task)
        quat = mul_wxyz(from_rotvec_wxyz(0.02 * a[3:]), s.ee_quat)
        robot.set_ee_target(pos, quat)                        # 5. non-blocking; osc_shm holds it at 1 kHz
        time.sleep(max(0.0, t0 + (k + 1) / 10 - time.monotonic()))   # 6. fixed-rate tick
        s = robot.get_state()                                 # 7. newest frame of the 100 Hz stream
```

Between two policy steps `osc_shm` applies `τ = Jᵀ[Kp e − Kd ẋ]` a hundred times
to the held target (zero-order hold); the IsaacLab replay reproduces exactly that
staircase, which is why the sysid transfers. `error_delta_pos` is set above the
largest single step so the controller never clips — the sim's task impedance has
no clip either. Targets are absolute poses in the base frame, quaternions
**wxyz**; gains and clamps persist across `move_to` and controller restarts.
Control law, safety chain and timing: [docs/architecture.md](architecture.md);
every client method and config key: [docs/usage.md](usage.md).

## 3. System identification

From a real excitation run to a PhysX arm that tracks it, in four commands.
Excitation design, parameter bounds and the loss: [docs/sysid.md](sysid.md).

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
overrides). Results on our arm: [Reference results](sysid.md#reference-results).

Everything you can program against — scripts, the Python client, the ZMQ
protocol, shared memory, the C++ binaries, config, data files, IsaacLab tasks —
is catalogued in [Interfaces](interfaces.md).

## Repository layout

```
src/                 C++: osc_shm (1 kHz controller), move_to (reset), read_* utilities, shm_layout.h
python/frankatwin/   daemon, FrankaTwinClient (PC), LocalController (NUC), config, shm_layout,
                     doctor, excitation/ (multiband + chirp reference math)
config/robot.yaml    network, robot IP, gains, safety clamps, collision thresholds, payload
examples/            move_to (home / joints / EE pose), cart_impedance (sine / multiband / chirp), policy_loop
scripts/             gen_*_traj (write references), compare_sim_real, check_torque_limits, plot_ee_tracking
isaaclab_sysid/      self-contained IsaacLab extension: tasks, robot USD, sysid/replay scripts
tests/               shm ABI pinning test (C++ offsets vs numpy dtype)
docs/                installation · architecture · usage · sysid · data_format · troubleshooting
```
