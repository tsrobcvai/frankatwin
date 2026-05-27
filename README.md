# panda_control

Real-time Cartesian impedance control for the **Franka Research 3 (FR3)**, built
with a PC↔NUC split architecture and tuned for **sim-to-real transfer**. The
codebase ships with two excitation-trajectory families and a system-ID pipeline
that brings the IsaacLab simulator to within a few mm RMS of the real arm on
held-out trajectories.

## Highlights

- **1 kHz J<sup>T</sup> task-space impedance controller** running natively on the NUC via libfranka.
- **PC↔NUC** split: realtime control on the NUC; Python policies / data tooling on the PC.
- **POSIX shared memory + ZMQ** for low-latency cross-process state and command flow.
- **System identification** delivers IsaacLab joint-position RMSE ≈ 1-3 % of motion range across a held-out chirp (v3 best params on v4 chirp: 4.8e-4 rad²).
- Two excitation trajectories included: **v3** (multi-band sinusoid) and **v4** (UR5e-style linear chirp, adapted for Franka).

## Architecture

```
        PC (workstation)                            NUC (cabled to Franka FCI)                  Franka FR3
        ──────────────                              ───────────────────                         ──────────
                                       ZMQ REQ/REP @ port 5555
        examples/cart_impedance.py ◄──────────────────────────────►  panda_control.daemon
        examples/reset_home.py                                              │
                                       ZMQ PUB/SUB @ port 5556              │  spawns / supervises
        Python user code               ◄──────  state stream  ──────        ▼
                ▲                                                      ┌─── osc_shm ────┐  libfranka
                │ uses                                                 │  1 kHz J^T     │  ─────────►   FR3
                ▼                                                      │  impedance     │     control
        panda_control.remote_client                                    │  controller    │     loop
        (RemotePandaClient)                                            └────────────────┘
                                       POSIX shm (/panda_osc)               ▲
                                       state ring buffer +                  │  one-shot resets
                                       command seqlock                      ▼
                                                                       ┌─── move_to ────┐
                                                                       │  MotionGenerator│
                                                                       └────────────────┘
```

- **`osc_shm`** is the long-running 1 kHz Jacobian-transpose Cartesian impedance controller. It reads target pose / gains / safety clamps from POSIX shared memory and publishes joint + EE state into the same segment.
- **`move_to`** is a short-lived libfranka MotionGenerator used for joint-space resets. The daemon stops `osc_shm`, runs `move_to`, then restarts `osc_shm` (libfranka allows only one FCI session at a time).
- **`panda_control.daemon`** (Python) owns the shm segment, supervises both binaries, and bridges them to the PC over ZMQ.

## Repository Layout

```
src/                  C++ realtime binaries (NUC): osc_shm, move_to, read_current_{q,pose}
python/panda_control/ Python package: daemon, RemotePandaClient, shm bindings
examples/             reset_home.py, cart_impedance.py
scripts/              excitation trajectory generators (v3, v4) + sim-vs-real comparison
config/robot.yaml     FCI IP, NUC host, default gains, safety clamps
tests/                Python ↔ C++ shm layout contract test
```

## Installation

The NUC is the machine cabled to the FR3's FCI port; the PC is the workstation
running policies and data tooling. The PC never opens an FCI session.

### NUC (libfranka + Python)

```bash
# Prerequisites: libfranka built/installed system-wide (e.g. via deoxys
# franka-interface, or built from source), conda/python 3.10+, cmake >= 3.10.

git clone <repo> /home/nuc/Projects/panda_control
cd /home/nuc/Projects/panda_control

# Build the C++ binaries.
cmake -S . -B build
cmake --build build -j

# Confirm the four binaries are present.
ls build/osc_shm build/move_to build/read_current_q build/read_current_pose

# Install the Python package (editable). Pulls pyzmq, numpy, pyyaml.
pip install -e .

# Sanity-check the shm layout pinning.
python -m pytest tests/test_shm_layout.py -q

# Important: deoxys's franka-interface must NOT be running -- libfranka
# only grants one FCI session at a time.
pgrep -a franka-interface && echo "kill franka-interface before continuing"
```

### PC (Python only)

```bash
git clone <repo> ~/Projects/panda_control
cd ~/Projects/panda_control

# No C++ build needed on the PC.
pip install -e .

# Verify your robot.yaml points at the NUC.
grep nuc_host config/robot.yaml   # default: 172.16.0.1
```

## Quick Start

### 1. Start the daemon on the NUC (once per boot)

```bash
python -m panda_control.daemon --config config/robot.yaml
```

This binds `tcp://*:5555` (REQ/REP) and `tcp://*:5556` (state PUB @ 100 Hz),
claims the POSIX shm segment `/panda_osc`, and launches `osc_shm`.

### 2. Reset to home (joint-space position control)

```bash
# On the PC
python examples/reset_home.py
```

Internally: daemon stops `osc_shm` → spawns `move_to` (libfranka
MotionGenerator, min-jerk) → waits for arrival → restarts `osc_shm` anchored to
the new EE pose.

### 3. Task impedance control (no null-space term)

```bash
# Smoke test: 4 s of z-axis sine around the current pose.
python examples/cart_impedance.py

# v4 chirp excitation for sysid (recommended Franka operating point).
python examples/cart_impedance.py --mode chirp \
    --rate 50 --kp-pos 500 --kp-ori 30 --f1 0.7 \
    --err-delta-pos 0.15 --err-delta-rot 0.80 \
    --log data/run_$(date +%Y%m%d_%H%M%S).csv
```

The Cartesian impedance law is `F = K_p · (x_des - x) - K_d · v`, mapped through
`τ = J^T · F`, with `K_d = 2·√K_p` (critical damping by default). No explicit
null-space term — the J<sup>T</sup> formulation lets joint-space null motion
settle under gravity and joint damping.

### 4. Send a custom target from Python

```python
import numpy as np
import time
from panda_control.config import load_config
from panda_control.remote_client import RemotePandaClient

cfg = load_config()                          # reads config/robot.yaml
with RemotePandaClient(cfg) as robot:
    state = robot.wait_for_state(timeout_s=3.0)
    anchor_pos  = state.ee_pos               # (3,) world frame, meters
    anchor_quat = state.ee_quat              # (4,) wxyz

    # Drive a +5 cm z offset at 50 Hz for 2 s.
    dt = 0.02
    for k in range(int(2.0 / dt)):
        t = k * dt
        offset = np.array([0.0, 0.0, 0.05 * np.sin(2 * np.pi * t)])
        robot.set_ee_target(anchor_pos + offset, anchor_quat)
        time.sleep(dt)

    # State is also streamed; pull the latest frame.
    s = robot.get_state()
    print(f"q = {s.q}, ee_pos = {s.ee_pos}")
```

`RemotePandaClient` exposes `set_ee_target`, `set_gains`, `get_state` /
`wait_for_state`, `move_to_q`, `move_to_pose`, `enable` / `disable`, and `ping`.
All methods are non-blocking on the realtime path — `set_ee_target` writes a
seqlock-protected command frame to shm and returns immediately; `osc_shm`
consumes it on the next 1 kHz tick.

## System Identification

The sysid optimizer (CMA-ES over 29 parameters: armature + static / dynamic /
viscous friction × 7 joints + motor delay) lives in the
[uw-lab/IsaacLab](https://github.com/uw-lab/IsaacLab) tree under
`scripts/tools/sysid_franka_osc.py`. This repo ships the excitation generators
and the real-side runner; the optimizer and IsaacLab sim live next door.

### Excitation Trajectories

Both trajectories drive the same `cart_impedance.py` Cartesian impedance loop;
they differ only in the *reference* sent to the controller.

| | **v3** (`gen_excitation_traj.py`) | **v4** (`gen_chirp_traj.py`) |
|---|---|---|
| Spectrum | two-band sinusoid per axis (low ≈ 0.15-0.30 Hz + high ≈ 0.7-1.1 Hz) | linear chirp f<sub>0</sub>→f<sub>1</sub>, 0.1→1.5 Hz |
| Active DOFs | x, y, z + optional yaw / roll | x, y, z, r<sub>x</sub>, r<sub>y</sub>, r<sub>z</sub> (always-on, π/3 phase-staggered) |
| Amplitudes | 4 / 4 / 3 cm + 0.05 rad yaw/roll | 10 / 10 / 15 cm + 0.50 / 0.25 / 0.50 rad |
| Envelope | symmetric 2 s half-cosine | asymmetric 2 s up / 3 s down (linear) |
| Duration | 12 s | 8 s |
| Origin | this repo (step5d lineage) | adapted from UR5e [`omnireset/diffusion_policy/scripts/sim2real/collect_sysid_data.py`](https://github.com/uw-lab/omnireset) |

v4 follows the UR5e chirp shape exactly, with three Franka-specific deltas:

1. **f<sub>1</sub> halved 3.0 → 1.5 Hz** (0.7 Hz for the production operating point). Franka's wrist joints J5–J7 have a 12 N·m effort limit; at the UR5e chirp top frequency they saturate and `osc_shm` latches its abort clamp.
2. **kp lowered 1000 → 500 N/m, kp<sub>ori</sub> 50 → 30** for parity with the downstream policy controller.
3. **Per-tick safety clamps relaxed** (`error_delta_pos: 0.05 → 0.15 m`, `error_delta_rot: 0.30 → 0.80 rad`) because lower kp + UR5e amplitudes give larger steady-state tracking error than osc_shm's default envelope tolerates. Set at runtime via `cart_impedance.py --err-delta-pos / --err-delta-rot`.

### Workflow

1. Collect real excitation data with `examples/cart_impedance.py --mode {step5d|chirp}` — produces `<run>.csv` and `<run>.json` sidecar.
2. Optimize: `sysid_franka_osc.py --real_csv <run>.csv --real_sidecar <run>.json …` — produces `sysid_best_params.json`.
3. Validate by re-running the same real trajectory in IsaacLab and overlaying:
   ```bash
   # IsaacLab side
   python scripts/tools/apply_sysid_params.py \
       --best logs/sysid_franka/<ts>/sysid_best_params.json --invoke-replay \
       --real-csv <run>.csv --real-sidecar <run>.json \
       --replay-script scripts/tools/replay_python_csv_sim.py
   # panda_control side
   python scripts/compare_sim_real.py \
       --real-csv <run>.csv --sim-csv <run>_sim_sysid.csv --save
   ```

> Use `replay_python_csv_sim.py` (zero-order hold at 50 Hz) for
> `cart_impedance.py` logs. The other replay script
> (`replay_real_step5b_sim.py`) hardcodes a 1 kHz sim step and is only valid
> for legacy 1 kHz C++ collector CSVs — feeding it a 50 Hz log silently
> truncates the sim to 5 % of the trajectory length.

### Example Results

The CMA-ES sysid run on combined step5b/c/d (v3) real data produced the
parameters below. Validated on a held-out v4 chirp (kp = 500 / 30, f<sub>1</sub>
= 0.7 Hz, 8 s, UR5e amplitudes), the resulting IsaacLab simulation tracks the
real arm to 1-3 % of joint motion range — joint-position MSE = 4.8 × 10<sup>-4</sup>
rad², roughly **0.21× the in-distribution training score**, confirming that v3
parameters generalise across the broader-spectrum v4 chirp.

**v3 best parameters** (`logs/sysid_franka/20260525_145807/sysid_best_params.json`):

```json
{
  "best_score": 0.002247,
  "best_params_decoded": {
    "armature":          [0.382, 0.159, 0.157, 0.174, 0.239, 0.180, 0.060],
    "mu_static":         [0.728, 1.168, 0.591, 1.026, 1.632, 1.141, 1.065],
    "dynamic_ratio":     [0.399, 0.767, 0.573, 0.794, 0.566, 0.508, 0.467],
    "mu_dynamic":        [0.291, 0.895, 0.339, 0.815, 0.924, 0.580, 0.497],
    "mu_viscous":        [3.676, 2.289, 2.875, 2.367, 3.296, 0.788, 1.937],
    "motor_delay_steps": 1
  }
}
```

EE position — sim (orange) overlays real (blue) almost exactly; both lag the
target (dashed grey) by the same amount because the impedance controller is
identical in both worlds:

![EE position: target vs real vs sim](./docs/images/v3_sysid_v4chirp_position.png)

EE orientation quaternion — same story, all three components track in lockstep:

![EE quaternion: target vs real vs sim](./docs/images/v3_sysid_v4chirp_orientation.png)

Joint-space view (q on the left, dq on the right, 7 joints stacked) — each
sim trace sits on top of the real trace, individual-joint RMSE 12-30 mrad
(1.9 - 8.3 % of per-joint motion range):

![Per-joint q and dq: sim vs real](./docs/images/v3_sysid_v4chirp_joints.png)

## Configuration

`config/robot.yaml` keeps NUC and PC in agreement on:

- `network.nuc_host` / `cmd_port` / `state_port` — ZMQ endpoints.
- `robot.ip` — FCI IP, used only on the NUC side.
- `robot.init_q` — joint-space home configuration.
- `control.kp_pos` / `kp_ori` — default impedance gains (overridable per-run via `set_gains`).
- `control.error_delta_pos` / `error_delta_rot` — per-tick safety clamps inside `osc_shm` (overridable at runtime via `set_gains(error_delta_pos=…)`).

Override the config path with `PANDA_CONFIG=/path/to/local.yaml`.

## Safety Notes

- Always raise the user-stop and verify FCI mode before running anything.
- `deoxys`'s `franka-interface` daemon must NOT be running concurrently.
- `osc_shm` enforces per-tick `|e_pos|_inf` and `|e_ori|` clamps; if either is exceeded, it latches `τ = 0` and the arm holds against gravity. Lower-stiffness operating points (e.g. kp=500) and aggressive references may need looser clamps — see `cart_impedance.py --err-delta-pos / --err-delta-rot`.
- Cartesian peak speed convention (inherited from earlier validation): `|dx|_peak ≤ 0.30 m/s`, `|ω|_peak ≤ 0.50 rad/s`. The v4 chirp defaults exceed both — this is expected; libfranka's internal limits remain the hard safety boundary.
