# panda_control

From-scratch Franka controller stack. The plan is to grow this repo
**bottom-up, one small step at a time**, each step a tiny standalone program
that exercises libfranka's `robot.control(...)` at 1 kHz directly. Once the
C++ bottom layer is solid we add Python wrappers (via POSIX shared memory)
that mimic the structure of the UR controller in
`omnireset/diffusion_policy/diffusion_policy/real_world/rtde_interpolation_controller.py`.

We are *not* using deoxys's NUC/ZMQ/protobuf architecture, and we are *not*
using `panda-py`. The goal is full transparency from policy command down to
joint torque.

## Roadmap

| Step | Goal | Files |
|------|------|-------|
| **1**  | Joint PD hold at a CLI-given `q_des`. Validate 1 kHz, RT priority, dq noise. | `src/step1_joint_pd.cpp` |
| 2  | Task-space PD (no inertial decoupling). Hold a fixed EE pose. | `src/step2_task_pd.cpp` |
| 3  | Add null-space term for the 7-DOF redundancy. | `src/step3_task_pd_null.cpp` |
| 4  | OSC with inertial decoupling (full operational-space control). | `src/step4_osc.cpp` |
| 5  | Wrap step 4 with POSIX shared memory: C++ binary takes `target_pose / Kp / Kd` from shm. | `src/osc_shm.cpp` + `python/panda_control/shm_layout.py` |
| 6  | Python `PandaController(mp.Process)` mirroring `RTDEInterpolationController`. | `python/panda_control/controller.py` |
| 7  | sim2real evaluation harness mirroring `isaaclab_rollout/rollout_act.py` patterns. | `python/scripts/eval_real.py` |

Implemented so far: step 1 (joint hold), step 3 (explicit Coriolis for hold),
and step 4 (joint trajectory tracking).

## Step 1: Joint PD hold (== Joint PD + gravity comp)

Control law we write in `src/step1_joint_pd.cpp`:

```
tau_cmd = Kp * (q_des - q) - Kd * dq
```

Effective control law actually applied on the robot:

```
tau_motor = tau_cmd + g(q) + tau_friction
```

libfranka, when used via `robot.control(callback)` returning `franka::Torques`,
adds gravity `g(q)` and a friction-compensation term to whatever we return
(see `franka/robot.h`: *"joint-level torque commands **without gravity and
friction** by providing callback functions"*). It does **not**
auto-compensate Coriolis — that has to be computed and added explicitly via
`franka::Model::coriolis()`, which is what step 3 will do.

So **step 1 is, semantically, a "Joint PD + gravity comp" controller
already**; we just never compute `g(q)` ourselves. There is no separate
`step1.5` file.

Parameters / defaults:

- `Kp` scalar-broadcast to 7 joints.
- `Kd = 2*sqrt(Kp)` (critical damping) unless `--kd` overrides.
- `tau_cmd` is clamped per joint to Franka's nominal motor limits
  `[87, 87, 87, 87, 12, 12, 12]` Nm. The clamp applies to our PD output only,
  not to the gravity/Coriolis term that libfranka adds afterwards.

### Why these defaults

- `Kp` ramps from 0 to the target over `--ramp` seconds (default 1.5 s),
  so the initial torque is always 0 even if `q_des` and `q_init` happen to
  differ slightly. Gradual ramp lets you catch problems before they become
  spikes.
- The binary refuses to start if `|q_init - q_des|_inf > 0.25 rad`
  (~14.3 deg). Move the robot to `q_des` first (Franka Desk guiding mode is
  fine), then run.
- The CSV log records per-tick `(t, period_ms, q, dq, tau)` so you can
  inspect timing jitter and `dq` noise floor offline.

## Step 3: Joint PD + Coriolis compensation

Control law in `src/step3_joint_pd_coriolis.cpp`:

```
tau_cmd = Kp * (q_des - q) - Kd * dq + c(q, dq)
```

where `c(q, dq) = C(q, dq) * dq` comes from `franka::Model::coriolis()`.

Effective robot-side torque remains:

```
tau_motor = tau_cmd + g(q) + tau_friction
```

because libfranka automatically adds gravity and friction compensation in
torque mode. Step 3 only adds the missing Coriolis vector term explicitly.

How Coriolis is obtained in code:

- Create the model once before entering the 1 kHz callback:
  `franka::Model model = robot.loadModel();`
- Inside the callback, fetch:
  `std::array<double, 7> c_arr = model.coriolis(robot_state);`
- Map `c_arr` to Eigen and add it to `tau_cmd`.

Run Step 3:

```bash
./scripts/run_step3.sh
```

A/B testing without explicit Coriolis:

```bash
NO_CORIOLIS=1 ./scripts/run_step3.sh
```

Validation expectations for hold tests:

- `period (ms)` and `dq RMS` should look similar to step 1.
- `c  RMS (Nm) per j` should be near zero order (typically around `1e-3` Nm
  or lower) because hold tests have very small `dq`.
- `NO_CORIOLIS=1` vs default should look nearly identical in static hold.

## Step 4: Joint trajectory tracking

Control law in `src/step4_joint_traj.cpp` (same structure as step 3, but
`q_des` is now time-varying):

```
tau_cmd(t) = Kp * (q_des(t) - q) - Kd * dq + c(q, dq)
```

Design note (A vs current choice):

- We intentionally do **not** use form A
  `tau = Kp*(q_des-q) + Kd*(dq_des-dq) + c(q,dq)` in Step 4.
- We keep the literal Step 3 damping form `-Kd*dq` to match the planned
  control-law progression.
- Reason: under trajectory excitation, A's `+Kd*dq_des` term itself injects a
  phase-leading disturbance into the tracking-error dynamics, which can mask
  the specific effect we want to observe here when sweeping `Kp` for the
  stability envelope.

Trajectory is a single-joint sinusoid around `q_center`:

```
A_eff(t) = A * min(1, t / amp_ramp)
q_des_j(t) = q_center_j + A_eff(t) * sin(2*pi*f*t)
```

where only one selected joint is swept (`--joint`), all others hold `q_center`.

Run Step 4:

```bash
./scripts/run_step4.sh
```

Typical Kp sweep workflow:

```bash
KP=10  ./scripts/run_step4.sh
KP=30  ./scripts/run_step4.sh
KP=80  ./scripts/run_step4.sh
KP=150 ./scripts/run_step4.sh
```

CLI reference:

| Arg | Default | Notes |
|-----|---------|-------|
| `<robot_ip>` | (required) | e.g. `172.16.0.2` |
| `--q-center q1..q7` | (required) | 7 floats, radians |
| `--joint J` | `3` | 0-based sweep joint index (`3` = 4th joint/elbow) |
| `--amp A` | `0.10` | Sinusoid amplitude (rad) |
| `--freq F` | `0.25` | Sinusoid frequency (Hz) |
| `--kp K` | `50.0` | Scalar, broadcast to 7 joints |
| `--kd K` | `2*sqrt(kp)` | Scalar, broadcast to 7 joints |
| `--duration sec` | `8.0` | `0` = run until Ctrl+C |
| `--ramp sec` | `1.5` | Kp ramp-in |
| `--amp-ramp sec` | `--ramp` | Amplitude ramp-in duration |
| `--no-coriolis` | off | Disable explicit Coriolis for A/B |
| `--print-err-every N` | `100` | Print `|q_des-q|_inf` every N ticks (`0`=off) |
| `--log path` | (no log) | Writes per-tick CSV |

Validation checklist for Step 4:

1. Start with defaults (`KP=10`, `AMP=0.10`, `FREQ=0.25`, `DURATION=8`).
2. Confirm summary prints `tracking err RMS` and `tracking err |max|` columns.
3. Increase `KP` gradually; stable regime should reduce `tracking err RMS`.
4. If error spikes, oscillation appears, or runtime abort triggers, back off
   `KP` and record the previous stable value as that joint's envelope limit.

## Step 5: Jacobian-transpose Cartesian PD (no Lambda)

Control law in `src/step5_cart_pd.cpp` (position-only Cartesian control):

```
tau_cmd = J_p^T * (Kp * (x_des - x) - Kd * dx) + c(q, dq)
```

where:

- `x` is end-effector position in the base frame (meters)
- `J_p` is the translational Jacobian (`zeroJacobian(...).topRows<3>()`)
- `dx = J_p * dq`
- `c(q, dq)` comes from `franka::Model::coriolis()`

This is intentionally the non-decoupled Cartesian baseline before Step 6
(`Lambda`) and before orientation/nullspace terms.

Trajectory mode:

- Default hold: `AMP=0`, so `x_des = x_anchor` (captured at startup)
- Optional single-axis sinusoid (`AXIS=x|y|z`):
  - `x_des[axis] += A_eff * sin(2*pi*f*t)`
  - `A_eff = A * min(1, t / amp_ramp)`

Run Step 5 (safe defaults):

```bash
./scripts/run_step5.sh
```

Enable 1 cm scan on z:

```bash
AMP=0.01 AXIS=z ./scripts/run_step5.sh
```

CLI reference:

| Arg | Default | Notes |
|-----|---------|-------|
| `<robot_ip>` | (required) | e.g. `172.16.0.2` |
| `--kp K` | `100.0` | Scalar Cartesian stiffness, broadcast to xyz |
| `--kd K` | `2*sqrt(kp)` | Scalar Cartesian damping |
| `--axis x|y|z` | `z` | Scan axis in base frame |
| `--amp A` | `0.0` | Sinusoid amplitude in meters (`0` = hold) |
| `--freq F` | `0.25` | Sinusoid frequency in Hz |
| `--duration sec` | `8.0` | `0` = run until Ctrl+C |
| `--ramp sec` | `1.5` | Kp ramp-in |
| `--amp-ramp sec` | `--ramp` | Amplitude ramp-in duration |
| `--no-coriolis` | off | Disable explicit Coriolis for A/B |
| `--print-err-every N` | `100` | Print Cartesian `|e|_inf` every N ticks (`0`=off) |
| `--log path` | (no log) | Writes per-tick CSV |

Validation and comparison workflow:

1. In a stretched pose, run hold mode (`AMP=0`) and confirm `abort: none`.
2. Run `AMP=0.01 AXIS=z` with the same `KP`; record `cart err RMS`.
3. Increase `KP` (`KP=200`, `KP=500`) and check if `cart err RMS` drops.
4. Move to a folded pose and repeat the same runs.
5. Compare stretched vs folded `cart err RMS` under identical gains; this is
   the Step 5 demonstration of configuration-dependent stiffness without
   inertial decoupling.

Safety notes specific to Step 5:

- Runtime abort if Cartesian tracking error `|x_des - x|_inf > 0.05 m`.
- Runtime abort if any joint goes outside nominal Panda limits.
- Peak desired Cartesian speed check: `amp * 2*pi*freq <= 0.3 m/s`.
- Keep `AMP` small on first runs (`0.005` to `0.01` m).

## Step 5b: 6D pose Jacobian-transpose Cartesian PD (no Lambda)

Control law in `src/step5b_cart_pose.cpp` (position + orientation):

```
tau_cmd = J^T * F_task + c(q, dq)
F_task  = [Kp_pos*(x_des - x) - Kd_pos*v ;
           Kp_ori*e_o         - Kd_ori*w]
```

where:

- `J` is the full 6x7 Jacobian from `zeroJacobian(...)`.
- `v = J_pos * dq`, `w = J_ori * dq`.
- `e_o = 2 * vec(q_des * q^{-1})` with shortest-path quaternion sign handling.
- `c(q, dq)` comes from `franka::Model::coriolis()`.

Trajectory mode:

- Position target is optional single-axis sinusoid around `x_anchor`.
- Orientation target is held at startup anchor (`R_des = R_anchor`).

Run Step 5b (safe defaults):

```bash
./scripts/run_step5b.sh
```

Run 1 cm z-axis sweep:

```bash
KP_POS=200 KP_ORI=20 AMP=0.01 AXIS=z ./scripts/run_step5b.sh
```

CLI reference:

| Arg | Default | Notes |
|-----|---------|-------|
| `<robot_ip>` | (required) | e.g. `172.16.0.2` |
| `--kp-pos K` | `100.0` | Scalar translational stiffness, broadcast to xyz |
| `--kd-pos K` | `2*sqrt(kp-pos)` | Scalar translational damping |
| `--kp-ori K` | `20.0` | Scalar orientation stiffness, broadcast to xyz |
| `--kd-ori K` | `2*sqrt(kp-ori)` | Scalar orientation damping |
| `--axis x|y|z` | `z` | Position sweep axis in base frame |
| `--amp A` | `0.0` | Sinusoid amplitude in meters (`0` = hold) |
| `--freq F` | `0.25` | Sinusoid frequency in Hz |
| `--duration sec` | `8.0` | `0` = run until Ctrl+C |
| `--ramp sec` | `1.5` | Gain ramp-in duration |
| `--amp-ramp sec` | `--ramp` | Amplitude ramp-in duration |
| `--no-coriolis` | off | Disable explicit Coriolis for A/B |
| `--print-err-every N` | `100` | Print `|e_pos|_inf` and `||e_o||` every N ticks (`0`=off) |
| `--log path` | (no log) | Writes per-tick CSV |
| `--sidecar path` | derived from `--log` | Per-run JSON sidecar (replace `.csv`->`.json`) |

CSV columns (per tick):

```
t_s, period_ms,
q1..q7, dq1..dq7,
x_x,x_y,x_z, dx_x,dx_y,dx_z,
quat_x,quat_y,quat_z,quat_w, wx,wy,wz,
x_des_{x,y,z}, dx_des_{x,y,z},
quat_des_{x,y,z,w},
e_x,e_y,e_z, e_ox,e_oy,e_oz,
tau_pd1..tau_pd7,    # pre-clamp computed torque = J^T * f_task + c_vec
tau_cmd1..tau_cmd7,  # post-clamp torque actually returned to libfranka
c1..c7               # explicit Coriolis vector (0 if --no-coriolis)
```

JSON sidecar (written once per run, in success path AND on exception):

- Identifies the run: `started_utc`, `csv_path`, `sidecar_path`, `controller`,
  `robot_ip`, `control_rate_hz`, `frame`, `rt_priority`.
- All CLI args under `args` (including auto-derived `kd_pos` / `kd_ori`).
- Initial state for sim replay: `q_init[7]`, `x_anchor[3]`, `q_anchor_xyzw[4]`.
  `null` if connect/readOnce failed before they were captured.
- `abort` block: `code`, `name`, `joint`, `value`, `time_s`. `code=0` /
  `name="none"` for clean finish.
- `summary` block (jitter, dq_rms, c_rms, pos/ori RMS and max errors) when
  the control loop ran at least one tick; `null` otherwise.
- `exception` (string or `null`) and `ended_normally` (bool).

Sim2real workflow with these logs:

1. Real run produces `step5b_<ts>.csv` + `step5b_<ts>.json`.
2. Sim replay reads sidecar JSON: initialize the sim arm to `q_init`, verify
   `frame` and `args`.
3. Sim controller streams `(t_s, x_des_*, quat_des_*)` from the CSV at
   `control_rate_hz`. The same control law (pos+ori PD, no Lambda) keeps
   the comparison fair.
4. Sim records its own actual EE pose; align with real `(x_*, quat_*)`
   columns by `t_s` for direct sim-vs-real residuals.

Safety notes specific to Step 5b:

- Runtime abort if Cartesian tracking error `|x_des - x|_inf > 0.05 m`.
- Runtime abort if orientation error `||e_o||_2 > 0.30 rad`.
- Runtime abort if any joint goes outside nominal Panda limits.
- Peak desired Cartesian speed check: `amp * 2*pi*freq <= 0.3 m/s`.

Sweep and visualization workflow:

```bash
# 1) Hold test (no position excitation)
KP_POS=100 KP_ORI=10 AMP=0.0 DURATION=5 ./scripts/run_step5b.sh

# 2) Small-amplitude Cartesian sweep
KP_POS=200 KP_ORI=20 AMP=0.01 AXIS=z ./scripts/run_step5b.sh

# 3) Increase translational gain
KP_POS=500 KP_ORI=20 AMP=0.01 AXIS=z ./scripts/run_step5b.sh

# 4) Plot desired vs actual EE trajectory and tracking errors
python scripts/plot_step5.py --save --show
```

For stretched-vs-folded comparison, run the same commands in both
configurations and compare plotted `||e_pos||` plus RMS/max metrics.

## Prerequisites

1. **PREEMPT_RT kernel** on the machine that runs the binary
   (same machine that has the direct ethernet link to the FCI port).
   Check with `uname -a` &mdash; the kernel string should contain `PREEMPT_RT`
   (or `-rt`).
2. **libfranka** installed and matching the robot's FCI firmware. Either:
   - System-installed, so that `find_package(Franka)` resolves; OR
   - Reuse the libfranka already built inside the deoxys repo
     (`isaaclab_rollout/deoxys_control/deoxys/libfranka`) by passing its
     install/build prefix as `-DCMAKE_PREFIX_PATH=...` at configure time.
3. **Eigen3** (`sudo apt install libeigen3-dev`).
4. **FCI license active** on the controller, joint brakes released, blue
   robot LED, **emergency stop within reach.**
5. **Real-time scheduling permission** for the user. Either:
   - Add user to a `realtime` group with `/etc/security/limits.conf` entries:
     ```
     @realtime  -  rtprio   99
     @realtime  -  memlock  unlimited
     ```
   - Or grant the binary `sudo setcap 'cap_sys_nice=eip' build/step1_joint_pd`
     after building.

If the binary cannot set `SCHED_FIFO` it prints a warning and continues
without RT priority &mdash; useful for smoke tests off-robot, but jitter
numbers will be meaningless.

## Build

```bash
cd /home/tao/Projects/panda_control
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

If `find_package(Franka)` cannot find libfranka, point CMake at where
libfranka was installed/built, e.g.:

```bash
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/path/to/libfranka/install
```

## Run

The wrapper script applies safe defaults (`Kp=10`, `duration=3 s`):

```bash
# 1. Move the robot to q_des manually (use Franka Desk guiding mode).
# 2. Then:
./scripts/run_step1.sh
```

Or call the binary directly with custom args:

```bash
./build/step1_joint_pd 172.16.0.2 \
    --q-des 0 -0.785398 0 -2.356194 0 1.570796 0.785398 \
    --kp 50.0 \
    --duration 30 \
    --ramp 1.5 \
    --log data/step1_$(date +%Y%m%d_%H%M%S).csv
```

CLI reference:

| Arg | Default | Notes |
|-----|---------|-------|
| `<robot_ip>` | (required) | e.g. `172.16.0.2` |
| `--q-des q1..q7` | (required) | 7 floats, radians |
| `--kp K` | `50.0` | Scalar, broadcast to 7 joints. Range checked `[0, 2000]`. |
| `--kd K` | `2*sqrt(kp)` | Scalar, broadcast to 7 joints. |
| `--duration sec` | `30.0` | `0` = run until Ctrl+C. |
| `--ramp sec` | `1.5` | Kp ramp-in time. |
| `--log path` | (no log) | If set, append one CSV row per tick. |

### Read current joint position (`q`)

Use this utility to print the robot's current 7 joint angles once:

```bash
./scripts/read_q.sh
```

Or with custom IP:

```bash
ROBOT_IP=172.16.0.2 ./scripts/read_q.sh
```

It prints:
- current `q` in radians;
- one copy-paste line for `step1_joint_pd --q-des`.

## Validation checklist (run after step 1)

1. **First run**: use the script defaults (`KP=10`, `DURATION=3`). Robot
   should remain visibly still. If it twitches or drifts, stop and check
   `data/step1_*.csv`.
2. **Console summary** should show:
   - `RT priority : yes`
   - `period (ms) mean / std` &asymp; `1.000 / < 0.10` on a PREEMPT_RT
     kernel. Much larger std indicates RT priority did not actually take
     effect or another RT task is starving the loop.
   - `dq RMS` per joint typically a few mrad/s &mdash; this is the velocity
     noise floor for later OSC tuning.
3. **Step up Kp progressively**: `KP=50`, `KP=200`. The robot should feel
   stiffer; jitter and `dq` RMS should not change much.
4. **Step up duration**: `DURATION=30`, `DURATION=120`. Watch for
   thermal/error issues over time.

## Layout

```
panda_control/
|-- CMakeLists.txt
|-- README.md
|-- .gitignore
|-- src/
|   |-- step1_joint_pd.cpp
|   |-- step3_joint_pd_coriolis.cpp
|   |-- step4_joint_traj.cpp
|   |-- step5_cart_pd.cpp
|   `-- step5b_cart_pose.cpp
|-- scripts/
|   |-- run_step1.sh
|   |-- run_step3.sh
|   |-- run_step4.sh
|   |-- run_step5.sh
|   |-- run_step5b.sh
|   `-- plot_step5.py
`-- data/
    `-- .gitkeep         (CSV logs land here)
```
