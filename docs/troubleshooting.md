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

### `joint_motion_generator_acceleration_discontinuity` at the start of `move_to`

**Symptom.** `move_to` (either mode) aborts almost immediately with
`control_command_success_rate: 1` — communication was perfect, so this is the
commanded trajectory, not the link.

**Cause.** The FCI requires the first command of a motion to equal the robot's
current *commanded* setpoint (`q_d`), not its *measured* position (`q`). The
vendored `MotionGenerator` seeds `q_start_` from `robot_state.q`. The two differ
by the tracking error — a fraction of a mrad under gravity sag, far more after a
previous motion aborted mid-flight — and closing that gap in one 1 ms tick is an
acceleration far above `kMaxJointAcceleration` (10 rad/s²). The trajectory
itself is gentle; only the first tick is the problem.

**Fix.** `src/move_to.cpp` hands the generator a `RobotState` whose `q` has been
replaced by `q_d`; pose mode likewise seeds from `O_T_EE_c` on the first tick
(`O_T_EE_c` is only populated once the loop is running, so `readOnce()` cannot
supply it). Do not "fix" this in `src/examples_common.cpp` — it is vendored
verbatim.

Enabling libfranka's rate limiter (`limit_rate=true`) looks like a fix for this
and is not: it clamps the offending step instead of removing it, and on this
setup it made things worse (`control_command_success_rate: 0`).

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

### `command not possible in the current mode ("Move")` from `move_to` / `osc_shm`

**Symptom.** A hand-run binary dies immediately on its first parameter command:

```
[move_to] franka::Exception: libfranka: Set Joint Impedance command rejected:
          command not possible in the current mode ("Move")!
```

**Cause.** Something else already owns the robot — almost always your own
`frankatwin.daemon`, whose `osc_shm` child is in its 1 kHz control loop. The FCI
accepts the second TCP connection but keeps the motion/parameter authority with
the first holder, so the failure surfaces several calls later, inside
`setDefaultBehavior()`, in a message that never mentions the real owner.

**Fix.** Don't run a second controlling session. Either drive the robot through
the daemon (`client.move_to_pose(...)` / `client.move_to_q(...)`, which stops
`osc_shm`, runs `move_to`, then restarts it), or stop the daemon first.

Since v0.2 `osc_shm` and `move_to` take a per-robot advisory lock
(`/tmp/frankatwin-fci-<ip>.lock`, see `src/fci_lock.h`) *before* connecting, so
the clash is now reported up front and names the holder:

```
[move_to] robot 172.16.0.2 is already held by another frankatwin session
          (pid=6494 exe=osc_shm).
```

with exit code **5**. `frankatwin doctor` prints the lock state as `fci lock`.
The lock is held on an open file descriptor, so the kernel releases it even if
the holder is SIGKILLed or segfaults — a stale lock file is never something you
need to delete by hand.

Read-only helpers (`read_current_q`, `read_current_pose`, `read_load`) and
`gripper_cmd` take no lock: the first three only `readOnce()`, and the gripper
server is a separate TCP endpoint (1338). All four work while the daemon runs.

The lock is advisory and only covers frankatwin's own binaries. If a *foreign*
FCI client holds the robot (`franka_ros`, `franka-interface`, or a Desk
operation), you still get the raw libfranka message — `move_to` appends a hint
pointing at `frankatwin doctor`, whose `fci clients` line lists them.

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
(`--err-delta-pos` on `python examples/cart_impedance.py`, or `set_gains`). The
chirp defaults need `0.15 m`. Only position can trigger this abort: the
orientation channel is pure impedance, so a large `|e_ori|` never stops a run.

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

### `conda create` hangs for minutes on `Solving environment`

Look for `Error while loading conda entry point: conda-libmamba-solver` in the
output: conda has fallen back to the classic solver, which can take tens of
minutes on the env in [Installation](installation.md#conda-environment-libfranka)
(18 min and counting on our NUC, against 1.6 s for the same spec with
`micromamba`). Usually a partial base-env upgrade left `libmamba` linked against
a `libarchive` that is no longer installed. Either repair base conda, or create
the env with `micromamba` — same channel, same specs, same result:

```bash
micromamba create -p $(conda info --base)/envs/frankatwin -y -c conda-forge \
    python=3.11 "libfranka=0.20" eigen cmake cxx-compiler pkg-config make \
    "sysroot_linux-64=2.28"
```

### `Incompatible library version (server version: N, library version: M)`

libfranka speaks a different FCI protocol than the robot. Nothing in the build
catches this — it surfaces when `osc_shm` (or `read_current_q`) opens a session.
Read the robot's system version and install the matching libfranka from the table
in [Installation → NUC](installation.md#conda-environment-libfranka) (system
≥ 5.9 → protocol 10 → libfranka 0.18–0.21; 5.7.2–5.8 → 0.15–0.17; 5.5–5.6 → 0.13).

### `cmake` does not print `frankatwin: prepending CONDA_PREFIX/lib`

The `frankatwin` conda env was not active when you configured, so CMake picked up
whatever libfranka / compiler the system has (or none). `conda activate frankatwin`,
`rm -rf build`, configure again ([Installation → NUC](installation.md#nuc)).

### `The C++ compiler is not able to compile a simple test program` (conda env)

`undefined reference to memcpy@GLIBC_2.14` / `secure_getenv@GLIBC_2.17` in the
CMake error log: the env's sysroot is conda-forge's default 2.12, older than the
glibc conda's own `libstdc++` was built against. Recreate the env with
`"sysroot_linux-64=2.28"` in the `conda create` line
([Installation](installation.md#conda-environment-libfranka)), then
`rm -rf libfranka/build build` — CMake caches the broken-compiler verdict.

### `undefined reference to fcntl64@GLIBC_2.28` when linking `osc_shm`

Same cause, one step later: Poco from conda-forge needs glibc 2.28 and the env's
sysroot is 2.17 or older. libfranka itself built because a shared library
tolerates unresolved symbols in its dependencies. Pin `sysroot_linux-64=2.28` as
above and rebuild both libfranka and FrankaTwin from clean build directories.

### Building libfranka: `Compatibility with CMake < 3.5 has been removed`

conda's CMake 4 refuses libfranka 0.13's `cmake_minimum_required(VERSION 3.4)`.
Add `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` to the libfranka configure line (already
in [Installation](installation.md#conda-environment-libfranka)).

### Building libfranka: `Could NOT find Eigen3 (missing: EIGEN3_INCLUDE_DIRS)`

libfranka 0.13's `FindEigen3.cmake` reads a legacy variable that Eigen 5 no longer
exports. Pin Eigen 3.4 in the env: `conda install -n frankatwin "eigen=3.4"`.

### Undefined `pinocchio::…` symbols at link time

libfranka ≥ 0.14 (robot system ≥ 5.7) computes its dynamics model with
[Pinocchio](https://github.com/stack-of-tasks/pinocchio), so linking `Franka::Franka`
needs it too. The conda-forge `libfranka` packages pull `libpinocchio` in, and
`CMakeLists.txt` adds `find_package(pinocchio REQUIRED)` automatically — seeing this
error means the build did not run inside the `frankatwin` env (previous entry).

### `liburdfdom_world.so.5.1: cannot open shared object file`

Some conda-forge `libfranka` builds (seen with 0.15.0) do not declare their urdfdom
dependency, so conda resolves urdfdom 6. Pin it: `conda install -n frankatwin "urdfdom=5.1"`.

### `GLIBCXX_3.4.29 not found` or `libboost_filesystem.so.1.xx.0: cannot open`

You're mixing conda libraries with the system toolchain, or running a
capability-enabled (`setcap`) binary, which ignores `LD_LIBRARY_PATH`. The conda
blocks in `CMakeLists.txt` handle both when `CONDA_PREFIX` is set at configure
time — reconfigure inside the env (`rm -rf build` first).

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
