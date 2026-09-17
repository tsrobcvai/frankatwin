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

### Changed
- `move_to --pose` duration: lower bound relaxed 1.5 s -> 0.5 s and the default
  lowered 5.0 s -> 2.0 s (`reset.pose_duration`). The bound is a blanket guard,
  not a physical limit -- the travel distance is unknown at parse time, so a
  short duration over a long travel can still trip a Cartesian reflex.
- `move_to` validates only the pacing knob its mode uses, and rejects the other
  one outright: `--duration` with `--q` (or `--speed-factor` with `--pose`) was
  previously range-checked and then silently ignored, which hid the fact that
  the requested pacing would not happen.
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
