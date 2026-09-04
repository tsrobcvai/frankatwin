# Usage

## Basic control

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

Modes:

| mode | reference | default duration | typical gains |
|---|---|---|---|
| `sine` | ±`--amp` (5 cm) z-sine at `--freq` (0.5 Hz) | 4 s | yaml defaults |
| `multiband` | SysID v3: two-band sinusoids on x/y/z + yaw/roll (`frankatwin.excitation.multiband`) | 12 s | `--kp-pos 200 --kp-ori 20` |
| `chirp` | SysID v4: 6-DOF linear chirp `--f0 0.1 → --f1 0.7` Hz, π/3 phase-staggered (`frankatwin.excitation.chirp`) | 8 s | `--kp-pos 500 --kp-ori 30 --err-delta-pos 0.15 --err-delta-rot 0.80` |

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

## Daemon (NUC)

```
python -m frankatwin.daemon [-c robot.yaml] [-v]
                            [--load-mass KG] [--load-com X Y Z] [--load-inertia i0 … i8]
```

| flag | meaning |
|---|---|
| `-c`, `--config PATH` | `robot.yaml` to use. Default: `$FRANKATWIN_CONFIG`, else the checkout's `config/robot.yaml`. |
| `-v`, `--verbose` | DEBUG logging **and** `osc_shm`'s own stdout/stderr passed through to the terminal (its banner, reflex messages, per-tick warnings). Without `-v` that output is captured and only shown if `osc_shm` dies. Use `-v` for the first runs and whenever something is odd. |
| `--load-mass KG` | Payload for `setLoad`; overrides `load.mass` for this run. `0` = don't call `setLoad`, keep the Desk-configured load. |
| `--load-com X Y Z` | Flange → payload COM [m]; overrides `load.com`. |
| `--load-inertia i0 … i8` | 3×3 about the COM, row-major [kg m²]; overrides `load.inertia`. When omitted and mass > 0 a small positive diagonal is filled in (libfranka rejects all-zero). |

Typical invocations:

```bash
python -m frankatwin.daemon -v                                   # first runs: see everything
python -m frankatwin.daemon                                      # quiet
python -m frankatwin.daemon -c ~/robots/robot_b.yaml             # another robot / gain profile
python -m frankatwin.daemon --load-mass 0.15 --load-com 0 0 0.05 # camera mounted on the flange
nohup python -m frankatwin.daemon > daemon.log 2>&1 &            # in the background, log to file
```

### What it does at start-up

1. Load the config; bind ZMQ `REP tcp://*:5555` (commands) and `PUB tcp://*:5556` (state).
2. Create and zero the shm segment `/frankatwin_osc`.
3. Launch `osc_shm <robot.ip> --shm-name … [--load-*] --collision-torque … --collision-cartesian …`
   and wait until it publishes state frames (libfranka session set-up, 0.5–2 s;
   up to 3 launch attempts).
4. Write the `control:` gains / clamps from `robot.yaml` into shm (on later
   restarts: the last values the client set).
5. Start the 100 Hz state publisher and the 0.5 s watchdog; log `daemon ready`.

The banner you should see with `-v`:

```
[osc_shm] robot_ip = 172.16.0.2
[osc_shm] shm_name = /frankatwin_osc (opened)
[osc_shm] pid      = 12345
[osc_shm] RT       = SCHED_FIFO                       # "non-RT" -> no rtprio; fix before real runs
[osc_shm] tau_rate = 800.000000 Nm/s (slew limit)
[osc_shm] load     = none (using Desk-configured load) # or: 0.15 kg, com=[0, 0, 0.05] m (gravity-compensated)
[osc_shm] collision = torque 100 Nm, cartesian 100 N/Nm (reflex thresholds)
[osc_shm] q_init    = …                               # joints at start
[osc_shm] x_anchor  = …                               # EE pose the controller holds until a client sends a target
[osc_shm] quat (wxyz) = …
[osc_shm] starting 1 kHz loop. SIGINT to stop.
INFO frankatwin.local_controller: osc_shm running; gains restored: kp=200.0/20.0 err_delta=0.050/0.300 enabled=True
INFO frankatwin.daemon: daemon ready, awaiting commands
```

A `WARN: setLoad FAILED` line means the payload is **not** compensated (bad
inertia tensor); `RT = non-RT` means the loop runs without real-time priority.

### While it runs

- Client commands (`set_ee_target`, `set_gains`, `move_to_*`, …) are executed
  one at a time; a `move_to_*` stops `osc_shm`, runs `move_to`, and restarts
  `osc_shm` at the new pose with the previous gains.
- Lines worth grepping for in the log:
  `watchdog: osc_shm restarted` (the controller died — a reflex, an RT overrun —
  and was relaunched; the preceding `osc_shm exited unexpectedly (code=…)`
  line carries libfranka's reason), `starting osc_shm (attempt 2/3)` (the
  transient `Move command aborted!` race after a reset, retried automatically).
- The state stream keeps flowing at 100 Hz whether or not a client is connected.

### Stopping

`Ctrl-C` (or `{"op": "shutdown"}` from a client) performs a controlled stop:
`osc_shm` ramps its torque to zero over a few ms, waits for the arm to settle,
exits; the daemon unlinks the shm segment and releases the ports. Don't
`kill -9` it — that skips the ramp and can trip a reflex on the next start.

### When to restart

- After editing `robot.yaml` (gains, clamps, collision thresholds, payload) —
  the daemon reads it once at start; no rebuild needed.
- After rebuilding `osc_shm` / `move_to` (`cmake --build build`).
- One daemon per robot: a second one fails to bind the ports. Stop any other
  FCI client (`franka-interface`, `franka_ros*`, `read_*` utilities) first —
  libfranka allows one session at a time (`python -m frankatwin.doctor` flags them).

## Scripts

Plain Python files — read them, copy them, run them with `python …`. All accept
`--config`; otherwise `$FRANKATWIN_CONFIG`, then the checkout's `config/robot.yaml`.

| script | where | does |
|---|---|---|
| `python -m frankatwin.daemon [-c robot.yaml] [-v] [--load-mass …]` | NUC | The daemon (below). |
| `python -m frankatwin.doctor [--role auto\|nuc\|pc]` | NUC, PC | Environment check: RT kernel, rtprio, binaries + `ldd` (libfranka / pinocchio), FCI port, competing FCI clients, daemon ping, state stream. Exit 1 on a hard failure, with a hint per line. Run it first on both machines. |
| `examples/move_to.py [--target-joints J1..J7 \| --target-ee x y z qw qx qy qz] [--speed] [--duration]` | PC | Position-controlled move via `move_to`: home (`robot.init_q`) by default, a joint configuration, or an EE pose (base frame, quaternion **wxyz**). Prints the held pose afterwards. |
| `examples/policy_loop.py [--hz 10] [--duration 16] [--pos-scale 0.005] [--rot-scale 0.02]` | PC | Fixed-rate policy on top of task impedance: read state → policy → Δpose target → `set_ee_target`. Ships a stand-in policy (10 cm up/down every 4 s). The closed-loop rollout skeleton. |
| `examples/cart_impedance.py --mode {sine,multiband,chirp} …` | PC | Run a scripted Cartesian reference at `--rate` Hz, log CSV + sidecar, print tracking RMS and torque headroom. `--dry-run` needs no robot. |
| `scripts/gen_excitation_traj.py` / `scripts/gen_chirp_traj.py --base-sidecar ref.json` | PC | Write a 1 kHz reference CSV + sidecar (for plotting / other collectors). The math is `frankatwin.excitation`. |
| `scripts/compare_sim_real.py --real-csv a.csv --sim-csv a_sim.csv [--save] [--show]` | PC | Overlay target / real / sim EE pose and per-joint q, dq; print RMS. |
| `scripts/check_torque_limits.py run.csv` | PC | Per-joint max `\|tau_J\|` vs 87/87/87/87/12/12/12 N·m from a `cart_impedance.py` log. |
| `scripts/plot_ee_tracking.py run.csv` | PC | Actual vs target per dimension. |
| `scripts/read_q.sh` | NUC | `read_current_q` wrapper (`ROBOT_IP=…`). Stop the daemon first. |
| `build/read_current_pose <ip> [out.json]` | NUC | Current EE pose as a sidecar for `--base-sidecar`. |
| `build/read_load <ip>` | NUC | Print `m_ee / m_load / m_total` as the robot sees them. |

## Client API (PC)

```python
from frankatwin import FrankaTwinClient, LocalController, RobotConfig, load_config
from frankatwin.quat import from_rotvec_wxyz, mul_wxyz, error_rotvec_wxyz   # wxyz helpers
```

`FrankaTwinClient(cfg, *, connect_timeout_s=3.0, verbose=False)` — context
manager. Raises `RuntimeError` if the daemon doesn't answer `ping`. `LocalController`
has the identical method set for in-process use on the NUC.

| method | behaviour |
|---|---|
| `wait_for_state(timeout_s=3.0) → RobotState` | Block until the first state frame arrives. |
| `get_state(*, fresh=False) → RobotState \| None` | Latest cached frame (100 Hz stream). `fresh=True` does a synchronous round-trip. |
| `get_state_history() → list[RobotState]` | Up to `network.state_cache` (256) most recent frames. |
| `set_ee_target(pos[3], quat[4] wxyz)` | New impedance setpoint in the base frame. Non-blocking; held until the next call. |
| `set_gains(kp_pos, kp_ori, kd_pos, kd_ori, error_delta_pos, error_delta_rot)` | Any `None` keeps the current value. `kd_*=0` → auto `2√kp`. `error_delta_*=0` disables clamp + abort. |
| `enable()` / `disable()` | `disable` zeroes the impedance torque (gravity comp stays); the slew limiter ramps it. |
| `move_to_q(q[7], speed_factor=None)` | Blocking (≤ 60 s). Stops `osc_shm`, runs `move_to --q`, restarts `osc_shm` anchored at the new pose; gains/clamps are preserved. `speed_factor ∈ (0, 0.5]`, default from yaml. |
| `move_to_pose(pos, quat wxyz, duration=None)` | Blocking. libfranka `CartesianPose`, `duration ∈ [1.5, 20]` s. |
| `close()` | Also called by `__exit__`. |

`RobotState` fields: `timestamp_s`, `q[7]`, `dq[7]`, `ee_pos[3]`, `ee_quat[4]`
(**wxyz**), `tau[7]` (commanded, gravity excluded), `tau_J[7]` (measured, gravity
included), `ee_linvel[3]`, `ee_angvel[3]` (base frame, `J q̇`), `seq`.

Quaternion convention: **wxyz everywhere on the wire and in `RobotState`.** The CSV
logs and IsaacLab use **xyzw**; `python examples/cart_impedance.py` converts.

### Notes

- Send targets **relative to the pose the controller is anchored at**. After
  `move_to_*` or a watchdog restart the anchor is the current pose.
- Gains, clamps and `enabled` persist across controller restarts
  (`move_to_*`, watchdog relaunch): the daemon restores them after every
  `osc_shm` start. Initial values come from `robot.yaml → control:`.
- A large jump in `set_ee_target` is a torque step. The slew limiter keeps it
  from tripping a reflex, but `Kp · Δx` still has to stay under the collision
  threshold and `τ_max` — ramp your targets.
- `error_delta_pos = 0.15 m` at `kp_pos = 500` allows 75 N of push. Lower the
  clamp when working near people or fixtures.

## Configuration reference (`config/robot.yaml`)

| key | default | meaning |
|---|---|---|
| `network.nuc_host` | `172.16.0.1` | Daemon address as seen from the PC. |
| `network.cmd_port` / `state_port` | 5555 / 5556 | REQ/REP and PUB ports. |
| `network.state_cache` | 256 | Client-side frame cache depth. |
| `robot.ip` | `172.16.0.2` | FCI address (NUC side). |
| `robot.init_q` | Franka home | `examples/move_to.py` default (home) target, 7 floats [rad]. |
| `control.kp_pos` / `kp_ori` | 200 N/m / 20 N·m/rad | Initial gains the daemon writes at startup; runtime override via `set_gains` (persists across restarts). Also the defaults of `cart_impedance.py --kp-*`. |
| `control.kd_pos` / `kd_ori` | `null` | `null` → `2√kp` (`osc_shm`'s auto rule). |
| `control.error_delta_pos` / `error_delta_rot` | 0.05 m / 0.30 rad | Initial per-tick error clamp + abort. `0` → pure impedance (as in sim). Runtime override via `set_gains(error_delta_*)` or `python examples/cart_impedance.py --err-delta-*`. |
| `collision.torque_threshold` / `cartesian_threshold` | 100 N·m / 100 N | `setCollisionBehavior` thresholds (all entries). |
| `paths.build_dir` | `build` | Where `osc_shm` / `move_to` live (relative to repo root). |
| `paths.shm_name` | `/frankatwin_osc` | POSIX shm name. |
| `reset.joint_speed_factor` | 0.2 | `move_to --q` speed, (0, 0.5]. |
| `reset.pose_duration` | 5.0 s | `move_to --pose` duration, [1.5, 20]. |
| `load.mass` / `com` / `inertia` | 0 / 0 / 0 | Extra payload for `setLoad`; see the comments in the file. |
| `gripper.enabled` | false | Reserved; the daemon does not drive the gripper. |

`FRANKATWIN_CONFIG=/path/to.yaml` overrides the default location.
