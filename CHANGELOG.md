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

### Changed
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
