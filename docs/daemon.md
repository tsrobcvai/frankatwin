# Daemon

<kbd>NUC</kbd> `python -m frankatwin.daemon` owns the shared-memory segment,
runs `osc_shm`, and serves the ZMQ command / state sockets.

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

## What it does at start-up

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

## While it runs

- Client commands (`set_ee_target`, `set_gains`, `move_to_*`, …) are executed
  one at a time; a `move_to_*` stops `osc_shm`, runs `move_to`, and restarts
  `osc_shm` at the new pose with the previous gains.
- `gripper_*` commands run `gripper_cmd` on a separate thread — the Franka Hand
  has its own connection, so `osc_shm` is untouched and the command loop stays
  free for `set_ee_target` while the fingers move. One gripper command at a
  time; `gripper_stop` aborts it (SIGINT → `Gripper::stop()`).
- Lines worth grepping for in the log:
  `watchdog: osc_shm restarted` (the controller died — a reflex, an RT overrun —
  and was relaunched; the preceding `osc_shm exited unexpectedly (code=…)`
  line carries libfranka's reason), `starting osc_shm (attempt 2/3)` (the
  transient `Move command aborted!` race after a reset, retried automatically).
- The state stream keeps flowing at 100 Hz whether or not a client is connected.

## Stopping

`Ctrl-C` (or `{"op": "shutdown"}` from a client) performs a controlled stop:
`osc_shm` ramps its torque to zero over a few ms, waits for the arm to settle,
exits; the daemon unlinks the shm segment and releases the ports. Don't
`kill -9` it — that skips the ramp and can trip a reflex on the next start.

## When to restart

- After editing `robot.yaml` (gains, clamps, collision thresholds, payload) —
  the daemon reads it once at start; no rebuild needed.
- After rebuilding `osc_shm` / `move_to` / `gripper_cmd` (`cmake --build build`).
- One daemon per robot: a second one fails to bind the ports. Stop any other
  FCI client (`franka-interface`, `franka_ros*`, `read_*` utilities) first —
  libfranka allows one session at a time (`python -m frankatwin.doctor` flags them).
