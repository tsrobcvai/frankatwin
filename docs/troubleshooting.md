# Troubleshooting

Start with `python -m frankatwin.doctor` on the machine that misbehaves: it
checks the environment half of this page (RT kernel, rtprio, binaries,
libfranka / pinocchio loading, FCI port, competing FCI clients, daemon ping,
state stream) and prints a hint per finding.

Each entry is a failure we hit, with the fix and where it lives in the code.

## Reflexes and aborts

### `motion aborted by reflex! ["controller_torque_discontinuity"]`

A setpoint step became a torque step above libfranka's 1000 N·m/s. The
per-joint slew limiter in `src/osc_shm.cpp` (`--max-torque-rate`, default
800 N·m/s) prevents it; if you disabled it (`<= 0`), don't.

### `joint_motion_generator_acceleration_discontinuity` at the start of `move_to`

The first command of a motion must equal the robot's commanded setpoint `q_d`,
not the measured `q`. `src/move_to.cpp` seeds the generator from `q_d` (pose
mode: from `O_T_EE_c` on the first tick); keep that if you touch it, and do not
enable libfranka's `limit_rate` as a substitute — it made this worse. Do not
edit `src/examples_common.cpp`; it is vendored verbatim.

### `cartesian_reflex` during contact / insertion

The factory collision thresholds (20 N) are below the force the controller
legitimately applies. Raise `collision.cartesian_threshold` /
`torque_threshold` in `robot.yaml` (shipped: 100), applied at `osc_shm` start.

### `command not possible in the current mode ("Reflex")` after any reflex

`osc_shm` calls `automaticErrorRecovery()` at start and the watchdog relaunches
it within 0.5 s, so one reflex needs no daemon restart. If it persists, the
robot is in a state recovery cannot clear (user stop pressed): release the stop
and re-enable FCI in Desk.

### `command not possible in the current mode ("Move")` from `move_to` / `osc_shm`

Something else owns the robot — almost always your own daemon. Drive the arm
through the daemon (`move_to_q` / `move_to_pose`) or stop the daemon first.
frankatwin's own binaries report the clash up front with the holder's pid and
exit code 5 (`src/fci_lock.h`; `doctor` shows it as `fci lock`). A foreign
client (`franka_ros`, `franka-interface`, Desk) still produces the raw
libfranka message; `doctor`'s `fci clients` line lists it. The `read_*`
helpers and `gripper_cmd` take no lock and run alongside the daemon.

### `Move command aborted!` right after `move_to`

A transient FCI hand-over race. The daemon retries the launch 3× at 0.5 s
(`_OSC_START_MAX_ATTEMPTS` in `local_controller.py`); nothing to do.

### `robot is still moving` / torque step on stop

Don't `kill -9` `osc_shm`. SIGINT / SIGTERM perform a controlled stop: the slew
limiter ramps τ to zero and the loop ends once `|τ|∞ < 0.05 N·m` and
`|q̇|∞ < 0.05 rad/s` (1 s cap).

### Tracking-error abort (`|e_pos|_inf > error_delta_pos`)

Only possible with `error_delta_pos > 0`; `robot.yaml` ships 0, which disables
the clamp and this abort. If you set a positive value: raise `kp`, slow the
reference, loosen the clamp, or set it back to 0 (`--err-delta-pos`,
`set_gains`). If the arm appears to start and stop repeatedly, a client is
still streaming targets from a stale anchor into a relaunched controller —
stop the client, not the daemon.

### Arm is softer than expected after a reset

Since 0.2.0 the daemon restores the last gains after every `osc_shm` start
(`move_to_*`, watchdog relaunch), so this should not happen; with an older
daemon, re-apply `set_gains(...)` after each reset. `watchdog: osc_shm
restarted` in the daemon log marks a mid-run restart.

## Communication

### Client hangs / `daemon did not reply within 5.0s`

The daemon is down or `network.nuc_host` / ports are wrong; `osc_shm` died and
the watchdog is mid-restart (daemon log with `-v`); or another client's
`move_to_*` is in progress — commands are serialised.

### `daemon error: log_start: log_start takes no 'path' any more`

The PC checkout is older than the NUC's: `git pull` on the PC (editable
install, nothing to reinstall). The reverse message, `log_start needs a
non-empty 'path'`, means the NUC is older: pull there and restart the daemon.
Keep both machines on one commit; the wire protocol is not versioned.

### `could not save the 1 kHz ring log` after a run

The daemon keeps the last finished log until the next `log_start`; fetch it
again from any client with `robot.log_save(path)`. The sidecar is already on
disk with the reason in `ring_log.save_error`.

### `no state frame received from daemon`

Firewall on 5556, or `osc_shm` never started (`controller_pid == 0` in
`ping`). Run the daemon with `-v` and look for the `[osc_shm]` banner.

### `communication_constraints_violation`

The 1 kHz loop missed a deadline. Check `RT = SCHED_FIFO` in the banner (grant
`cap_sys_nice` or `ulimit -r 99`), that the NUC runs `PREEMPT_RT`, that nothing
else loads the CPU, and that `--print-every` is off.

## Build

### `cmake` does not print `frankatwin: prepending CONDA_PREFIX/lib`, or undefined `pinocchio::…` symbols at link time

The `frankatwin` conda env was not active when you configured.
`conda activate frankatwin`, `rm -rf build`, configure again
([Installation → NUC](installation.md#nuc)).

### `The C++ compiler is not able to compile a simple test program`, or `undefined reference to fcntl64@GLIBC_2.28`

The env's sysroot is older than the glibc its libraries were built against.
Recreate the env with `"sysroot_linux-64=2.28"`
([Installation](installation.md#conda-environment-libfranka)), then
`rm -rf libfranka/build build` — CMake caches the verdict.

### `Incompatible library version (server version: N, library version: M)`

libfranka speaks a different FCI protocol than the robot. Read the robot's
system version and install the matching libfranka from the table in
[Installation → NUC](installation.md#conda-environment-libfranka) (system ≥ 5.9
→ libfranka 0.18–0.21; 5.7.2–5.8 → 0.15–0.17; 5.5–5.6 → 0.13).

### `tests/test_shm_layout.py` fails

`src/shm_layout.h` and `python/frankatwin/shm_layout.py` disagree. Mirror every
field change in both and bump `FRANKATWIN_SHM_VERSION`.

## Payload compensation

### Arm sags in z after mounting a camera / tool

Set `load.mass` / `load.com` in `robot.yaml` or pass `--load-mass` /
`--load-com` to the daemon; `build/read_load <ip>` (daemon stopped) shows what
the robot assumes. Without a scale, bracket the mass: with 0 kg our arm sagged
−27 mm, with 0.3 kg it lifted +27 mm, so the true value was the midpoint,
0.15 kg.

### `Set Load command rejected: invalid argument!`

The inertia tensor is all zeros or not positive-definite. Leave `load.inertia`
at zero (the daemon fills a small positive diagonal) or give a valid tensor.
`osc_shm` continues **uncompensated** after a rejected load — read the banner.

## Sysid

### Sim joints j1 / j5 stay flat while real ones move

Your excitation has no rotation. Both shipped designs rotate by default, so
`--amp-yaw`, `--amp-rx`, `--amp-ry` (or `--amp-rx/ry/rz` for the chirp) were
zeroed. Put them back.

### Replay is 20× too short / truncated to 5 %

A 50 Hz log went through a 1 kHz replay path. Use `replay_python_csv_sim.py`
(zero-order hold over the real timestamps).

### `No module named 'compare_sim_real'`, or a flag the docs show is not recognised

IsaacLab is running an older copy of the scripts. Re-run
`install_into_isaaclab.sh` after pulling frankatwin.

### `the replay is written, but the comparison cannot run`

`pip install pandas matplotlib` in the IsaacLab env, then run the
`compare_sim_real.py` line the message prints; the sim CSV is already there.

### One run's `w*L of best` stays far above the other's during the fit

That run is under-fit. Raise its weight in `--traj_weights w1,w2` and refit.

### Out of GPU memory when the fit creates its envs

The sim holds `--num_envs × runs` envs (512 for the documented single run, 1024
with the high band). Lower `--num_envs`; CMA-ES works at any population size.

### `usd_path` not found when launching the IsaacLab scripts

Run them from the IsaacLab root; the task configs reference
`./source/isaaclab_assets/data/Robots/Franka/franka_mimic.usd` relative to it.

## Operational rules

- One *controlling* FCI client at a time: stop `franka-interface` (deoxys) and
  `franka_ros*` before starting the daemon. The `read_*` helpers and
  `gripper_cmd` take no FCI lock and may stay.
- Release the user stop and confirm FCI mode in Desk before every session.
- Stop the daemon before mounting or removing anything on the flange.
- Log every run on hardware; the sidecar's `summary.tau_J_limit_frac` tells you
  how close you came to the actuator limits.
