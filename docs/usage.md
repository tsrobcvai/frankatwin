# Usage

## Daemon (NUC)

```bash
python -m frankatwin.daemon [--config robot.yaml] [-v]
                            [--load-mass KG --load-com X Y Z [--load-inertia i0..i8]]
```

Binds `tcp://*:5555` (REQ/REP) and `tcp://*:5556` (PUB, 100 Hz), creates
`/frankatwin_osc`, starts `osc_shm`. `Ctrl-C` performs a controlled stop.

`--load-*` override the `load:` block for this run only. Mass 0 (default) means
"don't call `setLoad`, keep what Desk has". Restart the daemon after changing
gains/clamps/collision values in the yaml — no rebuild needed.

## Client API (PC)

```python
from frankatwin import FrankaTwinClient, LocalController, RobotConfig, load_config
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
| `move_to_q(q[7], speed_factor=None)` | Blocking (≤ 60 s). Stops `osc_shm`, runs `move_to --q`, restarts `osc_shm` anchored at the new pose. `speed_factor ∈ (0, 0.5]`, default from yaml. |
| `move_to_pose(pos, quat wxyz, duration=None)` | Blocking. libfranka `CartesianPose`, `duration ∈ [1.5, 20]` s. |
| `close()` | Also called by `__exit__`. |

`RobotState` fields: `timestamp_s`, `q[7]`, `dq[7]`, `ee_pos[3]`, `ee_quat[4]`
(**wxyz**), `tau[7]` (commanded, gravity excluded), `tau_J[7]` (measured, gravity
included), `ee_linvel[3]`, `ee_angvel[3]` (base frame, `J q̇`), `seq`.

Quaternion convention: **wxyz everywhere on the wire and in `RobotState`.** The CSV
logs and IsaacLab use **xyzw**; `cart_impedance.py` converts.

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
- A large jump in `set_ee_target` is a torque step. The slew limiter keeps it
  from tripping a reflex, but `Kp · Δx` still has to stay under the collision
  threshold and `τ_max` — ramp your targets.
- `error_delta_pos = 0.15 m` at `kp_pos = 500` allows 75 N of push. Lower the
  clamp when working near people or fixtures.

## Examples

| script | what it does |
|---|---|
| `examples/reset_home.py [--speed 0.2]` | `move_to_q(init_q)`. |
| `examples/move_to_q.py --q q1..q7 [--speed]` | Move to an arbitrary joint configuration and read back. |
| `examples/lift_ee.py [--height 0.01] [--duration 2]` | Ramp the z-target up from the current pose. Good first motion test. |
| `examples/cart_impedance.py --mode {sine,multiband,chirp} …` | The workhorse: runs a scripted Cartesian reference at `--rate` Hz, logs `--log run.csv` + `run.json` sidecar, prints tracking RMS and torque headroom. `--dry-run` builds and checks the reference without a robot. |

`cart_impedance.py` modes:

| mode | reference | default duration | typical gains |
|---|---|---|---|
| `sine` | ±`--amp` (5 cm) z-sine at `--freq` (0.5 Hz) | 4 s | yaml defaults |
| `multiband` | SysID v3: two-band sinusoids on x/y/z + yaw/roll (`scripts/gen_excitation_traj.py`) | 12 s | `--kp-pos 200 --kp-ori 20` |
| `chirp` | SysID v4: 6-DOF linear chirp `--f0 0.1 → --f1 0.7` Hz, π/3 phase-staggered (`scripts/gen_chirp_traj.py`) | 8 s | `--kp-pos 500 --kp-ori 30 --err-delta-pos 0.15 --err-delta-rot 0.80` |

## Scripts

| script | purpose |
|---|---|
| `scripts/gen_excitation_traj.py --base-sidecar ref.json` | Emit a 1 kHz v3 multiband target CSV + sidecar anchored on `ref.json` (from `read_current_pose` or a previous run). |
| `scripts/gen_chirp_traj.py --base-sidecar ref.json` | Same for the v4 chirp. |
| `scripts/compare_sim_real.py --real-csv a.csv --sim-csv a_sim.csv [--save] [--show]` | Overlay target / real / sim EE pose and per-joint q, dq; print RMS. |
| `scripts/check_torque_limits.py run.csv` | Per-joint max `\|tau_J\|` vs 87/87/87/87/12/12/12 N·m from a `cart_impedance.py` log. |
| `scripts/plot_ee_tracking.py run.csv` | Actual vs target per dimension. |
| `scripts/read_q.sh` | `read_current_q` wrapper (`ROBOT_IP=…`). Stop the daemon first. |
| `build/read_current_pose <ip> [out.json]` | Current EE pose as a sidecar for `--base-sidecar`. |
| `build/read_load <ip>` | Print `m_ee / m_load / m_total` as the robot sees them. |

## Configuration reference (`config/robot.yaml`)

| key | default | meaning |
|---|---|---|
| `network.nuc_host` | `172.16.0.1` | Daemon address as seen from the PC. |
| `network.cmd_port` / `state_port` | 5555 / 5556 | REQ/REP and PUB ports. |
| `network.state_cache` | 256 | Client-side frame cache depth. |
| `robot.ip` | `172.16.0.2` | FCI address (NUC side). |
| `robot.init_q` | Franka home | `reset_home.py` target, 7 floats [rad]. |
| `control.kp_pos` / `kp_ori` | 200 N/m / 20 N·m/rad | Defaults for `cart_impedance.py --kp-pos/--kp-ori`. `osc_shm` itself starts at 200 / 20; the daemon does not push these — call `set_gains` from your client. |
| `control.kd_pos` / `kd_ori` | `null` | `null` → `2√kp` (also `osc_shm`'s built-in rule). |
| `control.error_delta_pos` / `error_delta_rot` | 0.05 m / 0.30 rad | Reference values for the per-tick error clamp + abort. **Not applied automatically** — `osc_shm` starts with clamps off (pure impedance); enable via `set_gains(error_delta_*)` or `cart_impedance.py --err-delta-*`. |
| `collision.torque_threshold` / `cartesian_threshold` | 100 N·m / 100 N | `setCollisionBehavior` thresholds (all entries). |
| `paths.build_dir` | `build` | Where `osc_shm` / `move_to` live (relative to repo root). |
| `paths.shm_name` | `/frankatwin_osc` | POSIX shm name. |
| `reset.joint_speed_factor` | 0.2 | `move_to --q` speed, (0, 0.5]. |
| `reset.pose_duration` | 5.0 s | `move_to --pose` duration, [1.5, 20]. |
| `load.mass` / `com` / `inertia` | 0 / 0 / 0 | Extra payload for `setLoad`; see the comments in the file. |
| `gripper.enabled` | false | Reserved; the daemon does not drive the gripper. |

`FRANKATWIN_CONFIG=/path/to.yaml` overrides the default location.
