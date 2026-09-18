# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Franka Hand support: `gripper_cmd` C++ binary (libfranka `Gripper`, own
  connection on port 1338 — the arm controller keeps running), daemon ops
  `gripper_homing` / `gripper_move` / `gripper_grasp` / `gripper_stop` /
  `gripper_state` (asynchronous, polled), `FrankaTwinClient.gripper_open()` /
  `gripper_close()` / `gripper_homing()` / `gripper_stop()` / `gripper_state()`,
  `examples/gripper.py` (same `--open --width FRAC` / `--close --force N`
  semantics as the deoxys-based `control_gripper.py`), and real
  `robot.yaml → gripper:` keys (speeds, grasp force / width, epsilons).
  `doctor` checks the binary and the gripper port.
- Per-robot FCI lock (`src/fci_lock.h`): `osc_shm` and `move_to` take an
  advisory `flock` on `/tmp/frankatwin-fci-<ip>.lock` before connecting, so a
  second controlling session exits 5 with the holder's pid instead of dying
  inside libfranka on `Set Joint Impedance command rejected: command not
  possible in the current mode ("Move")`. Held on an open fd, so the kernel
  releases it even on SIGKILL — no stale locks. Read-only helpers and
  `gripper_cmd` take no lock and still run alongside the daemon. `doctor`
  reports the lock state as `fci lock`.
- `sysid_franka_osc.py --eval_params <file>`: roll out one fixed parameter set
  (a `sysid_best_params.json` or checkpoint) without CMA-ES; prints the loss per
  trajectory and writes `eval_rollout_<run>.csv` + `eval_result.json`.

### Fixed
- `set_gains`, `set_ee_target` and `enable` / `disable` failed under numpy 2.x
  with `only 0-dimensional arrays can be converted to Python scalars`. All
  three read the command block back to re-publish the fields they do not
  change, but indexed it as `prev["kp_pos"]` instead of `prev["kp_pos"][0]`:
  `read_command()` returns a shape-(1,) structured array, and numpy 2.0 turned
  `float()` on a shape-(1,) array from a DeprecationWarning into a TypeError.
  `snapshot_gains` / `restore_gains` already indexed correctly, which is why
  the existing tests passed. `tests/test_command_roundtrip.py` now covers the
  three methods.
- `doctor` reports numpy's version on the `python` line. Nothing requires a
  particular major version, but behaviour differs across them, so it belongs in
  the report people already paste when something is off.
- `sysid_franka_osc.py` advanced one 1 ms tick per CSV row, so a 50 Hz run was
  replayed 20× too fast and the loss compared sim time `k` ms against real time
  `20k` ms. The fit now uses the same zero-order-hold time base as
  `replay_python_csv_sim.py` (50-tick warmup, `round(Δt / 1 ms)` ticks per row,
  sample then re-target); a fit rollout matches the replay of the same
  parameters to < 0.003° per joint. Parameters fitted with the affected version
  should be re-fitted.

### Removed
- The 0.30 m/s Cartesian speed and 0.50 rad/s angular rate conventions, and the
  four pre-flight checks that warned against them (`cart_impedance.py`,
  `gen_excitation_traj.py`, `gen_chirp_traj.py`,
  `frankatwin.excitation.chirp`). The pair was inherited from the UR5e
  `collect_sysid_data.py` pipeline this excitation code was ported from, not
  from Franka: libfranka's own ceilings are 3.0 m/s and 2.5 rad/s
  (`franka/rate_limiting.h`), roughly 10x and 5x higher. They were also
  mis-calibrated against the configurations this repo ships and documents --
  `multiband` peaks at 0.3012 m/s and `chirp` at 0.457 m/s / 1.47 rad/s, so
  both known-good excitations warned about themselves on every run.

  The peak-rate diagnostics stay, now printed without a threshold: they
  describe the commanded trajectory (`dx_des` is its analytic derivative),
  which is worth seeing before committing an excitation to the robot.
  `osc_shm`'s per-tick tracking-error clamp is the safety net that actually
  runs.


### Changed
- **Breaking.** `examples/gripper.py --width` is metres, not a fraction of the
  stroke. `--width 0.42` used to mean 42 % (33.6 mm) and now means 42 cm, which
  is out of range and rejected with a message saying so. `--width-m` stays as a
  hidden alias. Nothing else in the project expressed a length as a fraction.
- `gripper.grasp_speed` 0.5 -> 0.1 m/s. The Franka Hand product manual (1.2,
  Technical Data) gives "Travel Speed (per finger) 50 mm/s"; both fingers move,
  so the width closes at up to 0.1 m/s, which is also what libfranka's own
  examples pass. 0.5 was 5x over and the firmware was silently capping it, so
  the documented "closes at up to 0.5 m/s" was never true. `move_speed` was
  already at the ceiling.
- `gripper.grasp_force` is documented as **30-70 N**, not `(0, 70]`: the manual
  states the continuous force is "adjustable 30-70 N", so asking for less than
  30 does not buy a gentler hold. Use `--close-width` to stop the jaws early
  instead. The value itself is unchanged and `gripper_cmd` still only rejects
  <= 0.

### Changed
- `robot.yaml` ships `control.error_delta_pos: 0` (pure impedance) instead of
  `0.05`, so the file finally agrees with `osc_shm`, which has always compiled
  in `0.0` for the same stated reason ("pure impedance to match the unclipped
  sim"). Until the daemon began restoring gains after every controller start
  (c5bac75), `osc_shm`'s 0 is what actually ran, and it is what the v3 sysid
  data was collected under; the restore silently flipped the clamp on.

  The clamp does two things at once: it bounds the controller's push to
  `kp_pos * error_delta_pos` and aborts the loop when the unclipped error
  exceeds it. At `kp_pos` 200 a 0.05 clamp allows 10 N, which cannot follow
  `multiband`'s 0.30 m/s reference, so `cart_impedance.py --mode multiband`
  aborted after ~70 ms and the watchdog relaunched into the same abort -- the
  arm visibly starting and stopping. The `config.py` fallback moves to 0 as
  well. Set a positive value to get the clamp and the abort back.

### Changed
- **Breaking.** The orientation channel is pure impedance: `error_delta_rot` is
  gone everywhere — the per-tick `|e_ori|` clip and the orientation
  tracking-error abort in `osc_shm`, the `ShmCommand` field (the struct is now
  **112 B**, `enabled` at offset 104), the `set_gains(error_delta_rot=)`
  argument on `LocalController` / `FrankaTwinClient` and its daemon wire field,
  `robot.yaml`'s `control.error_delta_rot`, `--err-delta-rot` on
  `examples/cart_impedance.py` and `examples/policy_loop.py`, and the
  `ORI_TRACK_ABORT_RAD` pre-flight warnings in the excitation scripts. Position
  keeps its clamp and abort (`error_delta_pos`). Rebuild and restart the daemon:
  an old binary and a new one disagree on the command-block layout. The
  remaining orientation safety net is the per-joint torque clamp, the slew
  limiter, and libfranka's collision reflex and hard limits — with no cap on
  `Kp_ori · e_ori`, consider lowering `collision.torque_threshold` (100 N·m was
  chosen on the assumption that the controller's own push was bounded).
- **Breaking.** One pacing knob for both `move_to` modes: `--q-max-speed`, a
  per-joint velocity cap in rad/s, range (0, 1.25], default 0.5. It replaces
  `--speed-factor` and `--duration` on the binary, `speed_factor=` / `duration=`
  on `LocalController` / `FrankaTwinClient` (`q_max_speed=`), the `speed_factor`
  / `duration` wire fields (`q_max_speed`), `--speed` / `--duration` on
  `examples/move_to.py` (`--q-max-speed`), and `reset.joint_speed_factor` /
  `reset.pose_duration` in `robot.yaml` (`reset.q_max_speed`). The default
  reproduces the old behaviour exactly: 0.5 rad/s is `speed_factor` 0.2 against
  MotionGenerator's largest `dq_max_` (2.5), and the 1.25 ceiling is the old
  `speed_factor <= 0.5`. A `robot.yaml` still carrying the old keys falls back
  to the default rather than erroring.

  Motion time is now derived from the travel in both modes, so a longer move
  takes longer instead of moving faster -- which a fixed duration could not do.
  For `--pose` the cap is **approximate**: libfranka owns the IK, so `move_to`
  estimates the joint displacement from the Jacobian at the start pose
  (damped least squares) and sizes the min-jerk profile from that. The estimate
  degrades over large reorientations and near singularities; the binary says so
  on stdout, and the docs repeat it.
- Installation is conda-only: one `frankatwin` env per machine, libfranka from
  conda-forge matched to the robot's FCI protocol (system 5.9 → `libfranka=0.20`;
  a mismatch only shows up when a session is opened). `CMakeLists.txt` finds
  conda's boost of any version and only when Pinocchio is linked.
- `sysid_franka_osc.py` replays the trajectories in parallel env blocks:
  `--num_envs` is the CMA population, and the sim holds `num_envs × trajectories`
  envs. Each block starts from its run's `q_init` and uses its run's gains from
  the sidecar, so runs recorded under different gains (chirp at kp 500 / 30,
  multiband at 200 / 20) can be fitted together; the shared-gains check is gone.
- The optimizer steps through the new `FrankaTwinSysidEnv.physics_step()`
  (controller + physics only, without `DirectRLEnv.step()`'s dones / rewards /
  observations and their per-tick GPU→CPU sync) and scores only at the CSV
  sample instants. The env no longer fetches the mass matrix in
  `task_impedance` mode without nullspace, where the controller does not use it.

## [0.2.0] — 2026-09-03

First public release, renamed from the internal `panda_control` repository.

### Changed
- Package renamed `panda_control` → `frankatwin`; `RemotePandaClient` →
  `FrankaTwinClient`, `LocalPandaController` → `LocalController`,
  env var `PANDA_CONFIG` → `FRANKATWIN_CONFIG`, shm segment `/panda_osc` →
  `/frankatwin_osc`, IsaacLab tasks `Isaac-UW-Franka-*` → `Isaac-FrankaTwin-*`.
- `cart_impedance.py --mode step5d` → `--mode multiband`;
  `build_step5d_trajectory` → `build_multiband_trajectory`.
- `gen_excitation_traj.py` / `gen_chirp_traj.py`: `--base-sidecar` is required;
  outputs default to `data/`.
- `apply_sysid_params.py`: default `--replay-script` is the shipped
  `replay_python_csv_sim.py`.

### Fixed
- The daemon now restores gains, error clamps and `enabled` after every
  `osc_shm` start. Previously each restart (inside `move_to_q`/`move_to_pose`
  and, since the watchdog landed, any automatic relaunch) silently reset the
  controller to `osc_shm`'s built-ins (kp 200 / 20, clamps off). Initial
  values come from `robot.yaml → control:`.

### Removed
- Experiment scripts that depended on an unpublished IsaacLab task
  (`six_dof_pose_test`, `fixed_delta_pose_test`, `two_phase_smoke_test` and
  their compare tools). They remain on the `v0.1` branch.

### Added
- `python -m frankatwin.doctor`: environment / connectivity check with a
  hint per failing line (RT kernel, rtprio, binaries + ldd, FCI port, other
  FCI clients, daemon ping, state stream).
- `examples/policy_loop.py`: fixed-rate policy skeleton on task impedance;
  `frankatwin.quat`: wxyz quaternion helpers (product, rotvec, osc_shm-style
  orientation error).
- `examples/move_to.py`: one position-control script — home (default),
  `--target-joints`, or `--target-ee x y z qw qx qy qz` (replaces
  `reset_home.py` / `move_to_q.py`).
- `frankatwin.excitation` package: the reference math moved out of
  `scripts/` (no more `sys.path` hacks); `scripts/gen_*_traj.py` are now
  just the command line around it.
- Tests for config resolution, excitation builders and the CLI.
- Apache-2.0 license, third-party notices, citation metadata, CI, and the
  `docs/` set (installation, architecture, usage, sysid, data format,
  troubleshooting).

## [0.1.x] — 2026-05 … 2026-07 (internal)

- 1 kHz Jacobian-transpose Cartesian impedance controller (`osc_shm`) with
  POSIX-shm command/state interface, PC↔NUC ZMQ daemon, joint/pose `move_to`.
- Torque slew-rate limiter (fixes `controller_torque_discontinuity` reflex at
  setpoint jumps), controlled stop, configurable collision thresholds,
  payload `setLoad`, shm v3 with measured `tau_J`, osc_shm watchdog,
  Pinocchio linking for libfranka ≥ 0.14.
- CMA-ES system identification (armature, static/dynamic/viscous friction,
  motor delay) with v3 multi-band and v4 chirp excitation; self-contained
  IsaacLab extension.
