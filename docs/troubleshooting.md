# Troubleshooting

Start with `python -m frankatwin.doctor` on the machine that misbehaves — it covers the
environment half of this page (RT kernel, rtprio, binaries, libfranka /
pinocchio loading, FCI port, competing FCI clients, daemon ping, state stream)
and prints the matching hint.

Each entry below is a real failure we hit. Symptom → cause → fix, plus where the fix
lives in the code so you can audit it.

## Reflexes and aborts

### `motion aborted by reflex! ["controller_torque_discontinuity"]`

**Symptom.** `osc_shm` dies at the moment a new target is sent (often at a phase
boundary of a scripted motion, or right after `set_gains`); the state stream
freezes; `move_to_q` is refused with `command not possible in the current mode
("Reflex")`.

**Cause.** The Jᵀ impedance law turns a setpoint step into a torque step in one
1 ms tick. At an outstretched configuration a 7.5 N lateral step is ≈ 2.9 N·m on
j1 → 2900 N·m/s, above libfranka's 1000 N·m/s limit. It is *not* a velocity or
joint-limit problem — the arm can be at rest. "Settling" the arm before the step
does not help; the step is the problem.

**Fix.** Per-joint torque slew-rate limiter in `src/osc_shm.cpp`
(`--max-torque-rate`, default 800 N·m/s). In-phase motion is far below the
limit, so the limiter only shapes the few-ms transitions. If you disabled it
(`<= 0`), don't.

### `cartesian_reflex` during contact / insertion

**Cause.** libfranka's factory collision thresholds (20 N) are below the force the
controller legitimately applies: `kp_pos · error_delta_pos = 500 · 0.05 = 25 N`.

**Fix.** `collision.cartesian_threshold` / `torque_threshold` in `robot.yaml`
(default 100, applied via `setCollisionBehavior` at `osc_shm` start). Lower
them for a tighter crash guard if your working forces are small.

### `command not possible in the current mode ("Reflex")` after any reflex

`osc_shm` calls `automaticErrorRecovery()` at start, and the daemon watchdog
relaunches `osc_shm` within 0.5 s of it dying, so a single reflex no longer
requires a daemon restart. If you still see this, the robot is in a state the
recovery cannot clear (e.g. user stop pressed): release the stop / re-enable FCI
in Desk.

### `Move command aborted!` right after `move_to`

Transient FCI session hand-over race when `osc_shm` restarts immediately after
`move_to`. The daemon retries the launch 3× (0.5 s apart); see
`_OSC_START_MAX_ATTEMPTS` in `local_controller.py`.

### `robot is still moving` / torque step on stop

Returning `MotionFinished` with a zero-torque step trips the discontinuity reflex
and, if the arm is moving, this error. `osc_shm` performs a controlled stop:
the slew limiter ramps τ to zero and the loop ends only when `|τ|∞ < 0.05 N·m`
and `|q̇|∞ < 0.05 rad/s` (1 s cap). Don't `kill -9` the controller.

### Tracking-error abort (`|e_pos|_inf > error_delta_pos`)

Your reference moves faster than the impedance can follow at the current `kp`.
Either raise `kp`, lower the reference speed, or loosen the clamp
(`--err-delta-pos/--err-delta-rot` on `python examples/cart_impedance.py`, or `set_gains`). The
chirp defaults need `0.15 m / 0.80 rad` because the reference itself reaches
0.61 rad of orientation offset.

### Arm is softer than expected after a reset

`osc_shm` re-seeds its built-ins (`kp 200 / 20`, clamps off) every time it
starts. Since 0.2.0 the daemon restores the last gains/clamps after every start
(`move_to_*`, watchdog relaunch), so this should not happen; with an older
daemon, re-apply `set_gains(...)` after each reset. Grep the daemon log for
`watchdog: osc_shm restarted` to find mid-run restarts.

## Communication

### Client hangs / `daemon did not reply within 5.0s`

- The daemon is not running, or `network.nuc_host`/ports are wrong.
- `osc_shm` died and the watchdog is mid-restart — check the daemon log (`-v`).
- A `move_to_*` is in progress on another client; commands are serialised.

### `no state frame received from daemon`

The PUB socket isn't reaching you (firewall on 5556) or `osc_shm` never started
(`controller_pid == 0` in `ping`). Run the daemon with `-v` and look for the
`[osc_shm]` banner.

### `communication_constraints_violation`

The 1 kHz loop missed its deadline. Check `RT = SCHED_FIFO` in the banner (grant
`cap_sys_nice` or `ulimit -r 99`), that the NUC runs a `PREEMPT_RT` kernel, and that
nothing else is hammering the CPU. `--print-every` output in the control loop
also costs time; keep it off in production.

## Build

### Undefined `pinocchio::…` symbols at link time

libfranka ≥ 0.14 uses Pinocchio for dynamics. Point CMake at it:
`-DCMAKE_PREFIX_PATH="…;/path/to/pinocchio"`. See
[installation.md](installation.md#libfranka--014-needs-pinocchio).

### `GLIBCXX_3.4.29 not found` or `libboost_filesystem.so.1.82.0: cannot open`

You're mixing a conda-built libfranka with the system toolchain, or running a
capability-enabled binary that ignores `LD_LIBRARY_PATH`. The conda blocks in
`CMakeLists.txt` handle both when `CONDA_PREFIX` is set at configure time; if you
use a system libfranka, configure with conda deactivated.

### `tests/test_shm_layout.py` fails

The C++ struct and the numpy dtype disagree. Every field change in
`src/shm_layout.h` must be mirrored in `python/frankatwin/shm_layout.py` and
bump `FRANKATWIN_SHM_VERSION`. A version mismatch at runtime is a hard error
(`SharedMemoryAccess` refuses to attach).

## Payload compensation

### Arm sags in z after mounting a camera / tool

libfranka only compensates the mass configured in Desk plus what you pass to
`setLoad`. Set `load.mass/com` in `robot.yaml` or `--load-mass/--load-com` on the
daemon. `build/read_load <ip>` (daemon stopped) shows what the robot currently
assumes.

**Calibrating the mass without a scale.** Gravity *force* depends on mass only,
so bracket it with the z-offset of the same motion at two guesses: with 0 kg the
arm sagged −27 mm, with 0.3 kg it lifted +27 mm → the true value is the midpoint,
0.15 kg (the z residual then dropped to −1.3 mm). `com` only affects the torque
split, i.e. orientation behaviour; keep x/y at 0 unless rotations show a
directional residual.

### `Set Load command rejected: invalid argument!`

The inertia tensor is all zeros or not positive-definite. The daemon auto-fills a
small positive diagonal when you leave `load.inertia` at zero; if you set it
yourself, make it a valid tensor (`O(1e-4)` kg·m² diagonal is fine for a
camera). `osc_shm` treats a rejected load as a warning and continues
**uncompensated**, so read the banner.

## Sysid

### Sim joints j1 / j5 stay flat while real ones move

Your excitation has no rotation component. Use `--mode multiband` with non-zero
`--amp-yaw/--amp-roll`, or `--mode chirp`.

### Replay is 20× too short / truncated to 5 %

You fed a 50 Hz `python examples/cart_impedance.py` log to a 1 kHz replay path. Use
`replay_python_csv_sim.py` (zero-order-hold over the real timestamps).

### `usd_path` not found when launching the IsaacLab scripts

Run them from the IsaacLab root; the task configs reference
`./source/isaaclab_assets/data/Robots/Franka/franka_mimic.usd` relative to it.

## Operational rules

- One FCI client at a time: stop `franka-interface` (deoxys), `franka_ros*`,
  and any `read_*` utility before starting the daemon.
- Release the user stop and confirm FCI mode in Desk before every session.
- Stop the daemon before mounting or removing anything on the flange.
- Log every run on hardware (`--log`); the sidecar's `summary.tau_J_limit_frac`
  tells you how close you came to the actuator limits.
