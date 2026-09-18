# Interfaces

FrankaTwin exposes the same controller at several levels. Pick the lowest one
you need; each is stable and documented here.

| layer | where | you get | typical use |
|---|---|---|---|
| [Scripts](#scripts) | <kbd>NUC</kbd> <kbd>PC</kbd> | `python examples/*.py`, `python -m frankatwin.*` | resets, data collection, a first policy |
| [Python client](#python-client) | <kbd>PC</kbd> | `FrankaTwinClient` — pose targets, gains, state, resets | policy rollouts, teleop |
| [Daemon protocol](#daemon-protocol) | any language, <kbd>PC</kbd> → <kbd>NUC</kbd> | JSON over ZMQ | non-Python clients |
| [Shared memory](#shared-memory) | same host as `osc_shm`, any language | 1 kHz command / state segment | a controller-side client without the daemon |
| [C++ binaries](#c-binaries) | <kbd>NUC</kbd> | `osc_shm`, `move_to`, `read_*` | running the controller by hand |
| [Configuration](#configuration) | all | `config/robot.yaml` | gains, clamps, payload, network |
| [Data files](#data-files) | <kbd>PC</kbd> <kbd>SIM</kbd> | run CSV + sidecar, sim CSV, fitted parameters | analysis, sysid |
| [IsaacLab tasks](#isaaclab-tasks) | <kbd>SIM</kbd> | `Isaac-FrankaTwin-{Replay,Sysid}-v0` | replay, system identification |

Conventions that hold everywhere: positions in metres in the robot base frame,
quaternions **wxyz** on the Python / ZMQ / shm side (xyzw only inside CSV,
sidecar JSON and IsaacLab), torques in N·m, joints ordered J1…J7.

## Scripts

Plain Python files — read them, copy them, run them with `python …`. All accept
`--config`; otherwise `$FRANKATWIN_CONFIG`, then the checkout's `config/robot.yaml`.
The walkthrough is in [Usage](usage.md).

| script | where | does |
|---|---|---|
| `python -m frankatwin.daemon [-c robot.yaml] [-v] [--load-mass …]` | NUC | The daemon (below). |
| `python -m frankatwin.doctor [--role auto\|nuc\|pc]` | NUC, PC | Environment check: RT kernel, rtprio, binaries + `ldd` (libfranka / pinocchio), FCI port, competing FCI clients, daemon ping, state stream. Exit 1 on a hard failure, with a hint per line. Run it first on both machines. |
| `examples/move_to.py [--target-joints J1..J7 \| --target-ee x y z qw qx qy qz] [--q-max-speed]` | PC | Position-controlled move via `move_to`: home (`robot.init_q`) by default, a joint configuration, or an EE pose (base frame, quaternion **wxyz**). Prints the held pose afterwards. |
| `examples/policy_loop.py [--hz 10] [--duration 16] [--pos-scale 0.005] [--rot-scale 0.02]` | PC | Fixed-rate policy on top of task impedance: read state → policy → Δpose target → `set_ee_target`. Ships a stand-in policy (10 cm up/down every 4 s). The closed-loop rollout skeleton. |
| `examples/cart_impedance.py --mode {sine,multiband,chirp} …` | PC | Run a scripted Cartesian reference at `--rate` Hz, log CSV + sidecar, print tracking RMS and torque headroom. `--dry-run` needs no robot. |
| `examples/gripper.py --open [--width M] \| --close [--force N] [--close-width M] \| --homing \| --stop \| --state` | PC | Franka Hand via the daemon; the arm controller keeps running. `--width` is metres. Closing is a libfranka *grasp*: the object sets the width, `--force` (default 70 N) sets the hold. |
| `scripts/gen_excitation_traj.py` / `scripts/gen_chirp_traj.py --base-sidecar ref.json` | PC | Write a 1 kHz reference CSV + sidecar (for plotting / other collectors). The math is `frankatwin.excitation`. |
| `scripts/tools/compare_sim_real.py --real-csv a.csv --sim-csv a_sim.csv [--save] [--show]` | SIM | Overlay target / real / sim EE pose and per-joint q, dq; print RMS. |
| `scripts/check_torque_limits.py run.csv` | PC | Per-joint max `\|tau_J\|` vs 87/87/87/87/12/12/12 N·m from a `cart_impedance.py` log. |
| `scripts/plot_ee_tracking.py run.csv` | PC | Actual vs target per dimension. |
| `scripts/read_q.sh` | NUC | `read_current_q` wrapper (`ROBOT_IP=…`). Read-only; works while the daemon runs. |
| `build/read_current_pose <ip> [out.json]` | NUC | Current EE pose as a sidecar for `--base-sidecar`. |
| `build/read_load <ip>` | NUC | Print `m_ee / m_load / m_total` as the robot sees them. |

`build/osc_shm` and `build/move_to` are deliberately absent from this table:
they are the daemon's own workers, not a user interface. Drive them through
`examples/move_to.py` and the daemon. The three `read_*` binaries and
`gripper_cmd` above are the only C++ binaries meant to be run by hand, and all
of them coexist with a running daemon — the `read_*` ones only `readOnce()`,
and `gripper_cmd` uses the gripper's own TCP endpoint (1338).

## Python client

```python
from frankatwin import FrankaTwinClient, LocalController, RobotState, load_config
```

`FrankaTwinClient(cfg, *, connect_timeout_s=3.0, verbose=False)` connects to the
daemon (REQ + SUB), pings it, and caches the 100 Hz state stream. Use it as a
context manager. `LocalController` offers the same methods in-process on the
NUC (no ZMQ); scripts written against one run against the other.

| method | blocking | effect |
|---|---|---|
| `set_ee_target(pos[3], quat[4])` | no | New impedance setpoint (base frame, wxyz). Held until the next call. |
| `set_gains(kp_pos=, kp_ori=, kd_pos=, kd_ori=, error_delta_pos=)` | no | Any argument left `None` keeps its value. `kd_* = 0` → `2√kp`. `error_delta_pos = 0` disables the position clamp + abort; orientation is always unclamped. Persists across controller restarts. |
| `enable()` / `disable()` | no | `disable` zeroes the impedance torque (gravity compensation stays); the slew limiter ramps it. |
| `get_state(*, fresh=False)` → `RobotState \| None` | no (`fresh=True`: one round-trip) | Newest frame of the 100 Hz stream. |
| `get_state_history()` → `list[RobotState]` | no | Up to `network.state_cache` (256) recent frames. |
| `wait_for_state(timeout_s=3.0)` → `RobotState` | until a frame arrives | Use once after connecting / resetting. |
| `move_to_q(q[7], q_max_speed=None)` | yes (≤ 60 s) | Joint-space position move via `move_to`; `osc_shm` restarts at the new pose with gains preserved. `q_max_speed ∈ (0, 1.25]` rad/s, default `reset.q_max_speed` (0.5). Exact cap. |
| `move_to_pose(pos[3], quat[4], q_max_speed=None)` | yes | Cartesian position move via libfranka `CartesianPose`; same knob and range. **Approximate** — the joint speed is estimated from the Jacobian at the start pose, since libfranka owns the IK. |
| `gripper_open(width=None, speed=None, *, wait=True)` | yes (< 2 s) | `Gripper::move` to `width` [m], default `gripper.max_width`. `osc_shm` keeps running (the hand has its own connection). |
| `gripper_close(width=None, speed=None, force=None, epsilon_inner=None, epsilon_outer=None, *, wait=True)` (= `gripper_grasp`) | yes (< 2 s) | `Gripper::grasp`: fingers drive towards `width` (default −0.01 = past closure, so the object sets the resting width) and squeeze with `force` (default 70 N). Returns `{"result": is-within-epsilon, "state": {...}}`. |
| `gripper_homing(*, wait=True)` | yes (~6 s) | Calibrate the stroke; once after power-up or a finger change. |
| `gripper_stop()` | yes (quick) | Abort the motion in flight. |
| `gripper_state()` → dict | yes (one round trip to the hand) | `{"width", "max_width", "is_grasped", "temperature"}`. |
| `gripper_wait(seq=None, timeout_s=20)` → dict | until the command finished | Collect the output of a `wait=False` command. |
| `close()` | — | Also on `__exit__`. |

`RobotState` (dataclass):

| field | shape | meaning |
|---|---|---|
| `timestamp_s` | float | seconds since `osc_shm` started (monotonic) |
| `q`, `dq` | (7,) | joint position [rad], velocity [rad/s] |
| `ee_pos` | (3,) | EE position, base frame [m] |
| `ee_quat` | (4,) | EE orientation, **wxyz**, unit norm |
| `ee_linvel`, `ee_angvel` | (3,) | EE velocity, base frame [m/s], [rad/s] (`J q̇`) |
| `tau` | (7,) | commanded impedance torque [N·m], gravity excluded |
| `tau_J` | (7,) | measured link-side torque [N·m], gravity included — compare against 87/87/87/87/12/12/12 |
| `seq` | int | 1 kHz frame counter |

Quaternion helpers in `frankatwin.quat` (`mul_wxyz`, `from_rotvec_wxyz`,
`to_rotvec_wxyz`, `error_rotvec_wxyz`, `wxyz_to_xyzw`, …). Full signatures:
[API reference](api.md).

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

## Daemon protocol

Any language that speaks ZMQ + JSON can drive the arm. Two sockets on the NUC
(`network.cmd_port` / `network.state_port`, default 5555 / 5556):

**Commands — REQ/REP, one JSON object per request.**
Request `{"op": "<name>", ...}`; reply `{"ok": true, ...}` or
`{"ok": false, "error": "<message>"}`. Commands are executed one at a time.

| `op` | request fields | reply fields | blocking |
|---|---|---|---|
| `ping` | — | `pid` (osc_shm pid, `0` = controller not running) | no |
| `set_ee_target` | `pos: [x, y, z]`, `quat: [w, x, y, z]` | — | no |
| `set_gains` | any of `kp_pos`, `kp_ori`, `kd_pos`, `kd_ori`, `error_delta_pos` | — | no |
| `enable` / `disable` | — | — | no |
| `get_state` | — | `state`: object (fields as below) or `null` | no |
| `move_to_q` | `q: [7]`, optional `q_max_speed` | — | yes |
| `move_to_pose` | `pos: [3]`, `quat: [4] wxyz`, optional `q_max_speed` | — | yes |
| `gripper_homing` | — | `started: true`, `seq` | no — runs on a daemon thread |
| `gripper_move` | `width` [m], optional `speed` | `started: true`, `seq` | no — runs on a daemon thread |
| `gripper_grasp` | optional `width`, `speed`, `force`, `epsilon_inner`, `epsilon_outer` (defaults: `robot.yaml → gripper`) | `started: true`, `seq` | no — runs on a daemon thread |
| `gripper_stop` | — | `result` | yes (quick) |
| `gripper_state` | optional `fresh: true` | `busy`, `last` (output of the last finished command: `ok`, `cmd`, `result`, `stopped`, `state`, `seq`, or `error`), `state` (live readOnce, only with `fresh` and not busy) | `fresh`: one round trip to the hand |
| `shutdown` | — | `shutting_down: true` | no (daemon exits after replying) |

Gripper commands return as soon as the daemon has started them, so a policy
loop's `set_ee_target` is never held behind a 2 s grasp on the serial REQ/REP
socket. Poll `gripper_state` (without `fresh`) until `busy` is false and
`last.seq` matches — that is what the Python client's `wait=True` does. A
second command while one is running is refused (`gripper busy`); `gripper_stop`
aborts it.

**State — PUB, JSON, 100 Hz** (sent whenever a new 1 kHz frame exists):

```json
{"timestamp_s": 12.345, "seq": 12345,
 "q": [7], "dq": [7], "ee_pos": [3], "ee_quat": [4],
 "ee_linvel": [3], "ee_angvel": [3], "tau": [7], "tau_J": [7]}
```

Minimal raw client (this is all `FrankaTwinClient` does underneath):

```python
import json, zmq
ctx = zmq.Context()
req = ctx.socket(zmq.REQ); req.connect("tcp://172.16.0.1:5555")
sub = ctx.socket(zmq.SUB); sub.connect("tcp://172.16.0.1:5556"); sub.setsockopt(zmq.SUBSCRIBE, b"")

req.send_json({"op": "set_gains", "kp_pos": 500, "kp_ori": 30}); assert req.recv_json()["ok"]
state = json.loads(sub.recv())                                   # latest frame
req.send_json({"op": "set_ee_target", "pos": state["ee_pos"], "quat": state["ee_quat"]}); req.recv_json()
```

Clients should time out on REQ (the Python client uses 5 s, 60 s for `move_to_*`)
and rebuild the socket on timeout — ZMQ REQ sockets are strictly alternating.

## Shared memory

On the NUC, `osc_shm` and the daemon meet in one POSIX shm segment,
`/frankatwin_osc` (`paths.shm_name`). A process on the same host can use it
directly — that is how `LocalController` works.

| region | size | writer → reader | protocol |
|---|---|---|---|
| `ShmHeader` | 32 B | both | `magic`, `version` (3), `state_frames` (1024), `controller_pid`, `state_head` (atomic 64-bit) |
| `ShmCommand` | 112 B | client → `osc_shm` | **seqlock**: writer bumps `seq` to odd, writes, bumps to even; reader retries while `seq` is odd or changed. Fields: `target_pos[3]`, `target_quat[4]` wxyz, `kp_pos`, `kp_ori`, `kd_pos`, `kd_ori`, `error_delta_pos`, `enabled` |
| `ShmStateFrame[1024]` | 384 B each | `osc_shm` → client | **SPSC ring** indexed by `state_head`; frame `seq` validates a read. Fields: `seq`, `timestamp_s`, `q[7]`, `dq[7]`, `ee_pos[3]`, `ee_quat[4]`, `tau[7]`, `ee_linvel[3]`, `ee_angvel[3]`, `tau_J[7]` |

Total 393 368 B. The layout is an ABI: `src/shm_layout.h` (C, with
`static_assert`s on every offset) and `python/frankatwin/shm_layout.py`
(numpy dtypes) must agree, and `tests/test_shm_layout.py` compiles the header
to check they do. Any field change bumps `FRANKATWIN_SHM_VERSION`.

- **Python**: `SharedMemoryAccess(name="/frankatwin_osc", create=False).view`
  → `ShmView` with `read_command()`, `write_command(...)`, `latest_state()`,
  `last_k_states(k)`, `controller_pid`, `state_head`.
- **C / C++**: `#include "shm_layout.h"`; `panda_shm::cmd_write_begin / cmd_write_end`,
  `cmd_read`, `state_publish`, `state_read_latest`.

## C++ binaries

Built by CMake into `build/`; all but `gripper_cmd` need the sole FCI session
(stop the daemon first when running them by hand).

| binary | usage | notes |
|---|---|---|
| `osc_shm` | `osc_shm <robot_ip> [--shm-name NAME] [--init-shm] [--no-coriolis] [--print-every N] [--duration s] [--max-torque-rate Nm_per_s] [--load-mass kg] [--load-com x y z] [--load-inertia i0…i8] [--collision-torque Nm] [--collision-cartesian N]` | The 1 kHz controller. `--init-shm` creates the segment (omit under the daemon). Defaults: slew 800 N·m/s, no payload, collision 20 (the daemon passes `robot.yaml`'s 100). Stops cleanly on SIGINT/SIGTERM. |
| `move_to` | `move_to <robot_ip> --q J1…J7 [--q-max-speed 0.5]` or `move_to <robot_ip> --pose x y z qw qx qy qz [--q-max-speed 0.5]` | Blocking. One pacing knob for both modes; approximate for `--pose`. Exit 0 ok, 1 bad CLI, 5 robot busy, 10 `franka::Exception`, 11 other. |
| `read_current_q` | `read_current_q <robot_ip>` | Print `q` once. |
| `read_current_pose` | `read_current_pose <robot_ip> [out.json]` | EE pose as a sidecar for `--base-sidecar`. |
| `read_load` | `read_load <robot_ip>` | Print `m_ee / m_load / m_total` as configured. |
| `gripper_cmd` | `gripper_cmd <robot_ip> homing \| move --width W [--speed S] \| grasp --width W [--speed S] [--force F] [--eps-in E] [--eps-out E] \| stop \| state` | Franka Hand over libfranka's gripper server (port 1338 — independent of the FCI session, so it runs alongside `osc_shm`). Prints one JSON object: `ok`, `cmd`, `result` (libfranka's return), `stopped`, `state {width, max_width, is_grasped, temperature}`. SIGINT/SIGTERM in flight → `Gripper::stop()`. Exit 0 ran, 1 bad CLI, 10 `franka::Exception`. |

## Configuration

One file, `config/robot.yaml`, read by every process: `network` (daemon
address, ports, client cache), `robot` (FCI IP, home `init_q`), `control`
(initial gains and error clamps), `collision` (reflex thresholds), `paths`
(build dir, shm name), `reset` (move speeds), `gripper` (Franka Hand speeds,
grasp force and width), `load` (payload for `setLoad`).
Override with `--config` or `$FRANKATWIN_CONFIG`. Key-by-key reference:
[Configuration](configuration.md).

## Data files

| file | producer | consumer |
|---|---|---|
| `<run>.csv` + `<run>.json` | `cart_impedance.py --log` | sysid fit, replay, compare / plot scripts |
| `<run>_sim*.csv` + `.json` | `replay_python_csv_sim.py` | `compare_sim_real.py` |
| target CSV + sidecar | `gen_*_traj.py` | plotting, other collectors |
| `sysid_best_params.json` | `sysid_franka_osc.py` | `apply_sysid_params.py`, your own IsaacLab task |

Column-by-column schemas: [Data format](data_format.md).

## IsaacLab tasks

Installed into an IsaacLab checkout by `isaaclab_sysid/install_into_isaaclab.sh`
(package `isaaclab_tasks.direct.franka_sysid`, asset `franka_mimic.usd`, three
scripts under `scripts/tools/`):

| task id | env class | purpose | key cfg fields |
|---|---|---|---|
| `Isaac-FrankaTwin-Replay-v0` | `FrankaTwinReplayEnv` | Replay a target trajectory under the mirrored controller; log the sim state. | `control_mode` (`task_impedance` \| `osc`), `use_nullspace`, `q_init`, `traj_log_path`, gains in `ctrl` |
| `Isaac-FrankaTwin-Sysid-v0` | `FrankaTwinSysidEnv` | Same controller, 128 envs, `DelayedPDActuator` on the arm; `set_targets()` / `step_replay()` drive it from the optimizer. | as above + per-env armature / friction / delay written by the sysid script |

| script | role |
|---|---|
| `sysid_franka_osc.py --real_csv … --real_sidecar … [--num_envs 128] [--max_iter 40] [--traj_weights …]` | CMA-ES over 29 parameters → `logs/sysid_franka/<ts>/sysid_best_params.json` |
| `apply_sysid_params.py --best … [--invoke-replay --real-csv … --real-sidecar …] [--print-snippet]` | Replay with the fitted parameters, or print actuator-config overrides |
| `replay_python_csv_sim.py --real-csv … --real-sidecar … [--sysid-params …] [--gain-source sidecar\|env_cfg]` | ZOH replay of a 50 Hz log at 1 kHz |

Details and the parameterisation: [System identification](sysid.md).
