# Usage

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

### Minimal loop

```python
import numpy as np, time
from frankatwin import FrankaTwinClient, load_config

cfg = load_config()                       # config/robot.yaml or $FRANKATWIN_CONFIG
with FrankaTwinClient(cfg) as robot:
    robot.move_to_q(cfg.robot.init_q)     # optional reset
    s = robot.wait_for_state()
    p0, q0 = s.ee_pos.copy(), s.ee_quat.copy()
    robot.set_gains(kp_pos=500, kp_ori=30, error_delta_pos=0.15, error_delta_rot=0.80)
    dt = 0.02
    for k in range(int(4 / dt)):
        robot.set_ee_target(p0 + [0, 0, 0.05 * np.sin(2 * np.pi * 0.5 * k * dt)], q0)
        time.sleep(dt)
    robot.set_ee_target(p0, q0)
```

Things that bite:

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

## `cart_impedance.py` modes

| mode | reference | default duration | typical gains |
|---|---|---|---|
| `sine` | ±`--amp` (5 cm) z-sine at `--freq` (0.5 Hz) | 4 s | yaml defaults |
| `multiband` | SysID v3: two-band sinusoids on x/y/z + yaw/roll (`frankatwin.excitation.multiband`) | 12 s | `--kp-pos 200 --kp-ori 20` |
| `chirp` | SysID v4: 6-DOF linear chirp `--f0 0.1 → --f1 0.7` Hz, π/3 phase-staggered (`frankatwin.excitation.chirp`) | 8 s | `--kp-pos 500 --kp-ori 30 --err-delta-pos 0.15 --err-delta-rot 0.80` |

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
