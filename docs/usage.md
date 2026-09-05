# Usage

## Basic control

Two steps: bring the controller up on the NUC, then drive the arm from the PC.
Every command below runs inside the `frankatwin` conda env of that machine
([Installation](installation.md)).
Step 2 shows four usage examples (each script takes `--config robot.yaml`; the
two arm controllers they use are described in [Architecture](architecture.md)).

### Step 1 · Start the daemon

<kbd>NUC</kbd>

```bash
conda activate frankatwin
python -m frankatwin.doctor          # other FCI clients, missing binaries, unreachable FCI
python -m frankatwin.daemon -v
```

`doctor` is optional but cheap: it flags the things that make the daemon fail to
start (another libfranka session, a missing `build/osc_shm`, FCI not reachable).
The daemon launches `osc_shm` — the arm now holds its current pose under impedance
control — and keeps it alive. Leave the terminal open; the banner should show
`RT = SCHED_FIFO`, `tau_rate = 800 Nm/s`, the payload and collision settings,
then `daemon ready`. All flags (`--config`, payload overrides), the banner line
by line, what it logs while running, how to stop it and when to restart it:
[Daemon](daemon.md).

### Step 2 · Drive the arm from the PC

<kbd>PC</kbd> — first confirm the PC sees the daemon:

```bash
conda activate frankatwin
python -m frankatwin.doctor          # daemon ping on 5555, state stream on 5556
```

If it fails, `network.nuc_host` in `config/robot.yaml` is the usual culprit
([Troubleshooting](troubleshooting.md)). Then four usage examples: a one-shot
move, a scripted reference, a closed-loop policy, and the gripper. Each one says
what the arm will do before you run it. Examples 2 and 3 run under impedance
control (the arm is compliant — you can push it and it springs back); Example 1
is stiff position control, see its safety note.

#### Example 1 · Reset the arm

Script: [`examples/move_to.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/move_to.py)

One-shot position control (`move_to`). Home by default; prints the pose
`osc_shm` holds afterwards.

**What the robot does.** `osc_shm` stops; libfranka drives all seven joints to
the target along a min-jerk profile (for `--target-joints` / home the EE sweeps
an arc, *not* a straight line; for `--target-ee` it follows a straight 5th-order
path), at `--speed` × the joint speed limits — the default 0.2 takes 3–5 s from
a typical pose. Then `osc_shm` restarts and holds the new pose compliantly.

> **Safety.** This is stiff position control: the arm moves through whatever
> lies between its current pose and the target, and does not yield on contact.
> Before every move: clear the workspace of objects and people, check that the
> target is reachable and that the path does not cross the table or fixtures,
> and hold the user stop in your hand for the whole motion. Use `--speed 0.1`
> the first time you try a new target.

```bash
python examples/move_to.py                                                # home = robot.init_q
python examples/move_to.py --target-joints 0 -0.785 0 -2.356 0 1.571 0.785 --speed 0.2
python examples/move_to.py --target-ee 0.4 0.0 0.3  0 1 0 0 --duration 5
```

| flag | meaning |
|---|---|
| `--target-joints J1 … J7` | 7 absolute joint angles [rad] |
| `--target-ee x y z qw qx qy qz` | EE position [m] in the robot **base frame** (+x forward, +z up) and orientation as a **unit quaternion, wxyz** — `0 1 0 0` = tool pointing down. EE frame = the one configured in Desk (Franka Hand: TCP between the fingertips) |
| `--speed` | joint move: speed factor (0, 0.5]; default `reset.joint_speed_factor` |
| `--duration` | EE move: seconds in [1.5, 20]; default `reset.pose_duration` |

#### Example 2 · Track a scripted reference

Script: [`examples/cart_impedance.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/cart_impedance.py)

Continuous control (`osc_shm`) following a scripted EE reference at `--rate` Hz.

**What the robot does.** Starting from wherever it is, the EE oscillates ±5 cm
along z at 0.5 Hz for 4 s (`sine`), then returns to the start pose. `multiband`
and `chirp` are the sysid excitations: 6-DOF motion of up to ±10–15 cm and
±0.5 rad for 8–12 s with peak speeds around 0.5 m/s and visibly fast wrist
rotation — start them from a pose with at least 30 cm of free space in every
direction. Under impedance control the arm is compliant throughout.
Logs a per-tick CSV + sidecar and prints tracking RMS and torque headroom. This
is also the sysid data collector.

```bash
python examples/cart_impedance.py                                         # sine: ±5 cm z at 0.5 Hz for 4 s
python examples/cart_impedance.py --mode chirp --kp-pos 500 --kp-ori 30 \
    --err-delta-pos 0.15 --err-delta-rot 0.80 --log data/run.csv        # sysid v4 chirp, logged
python examples/cart_impedance.py --mode multiband --dry-run              # build + check the reference, no robot
```

| flag | meaning |
|---|---|
| `--mode sine \| multiband \| chirp` | reference: z-sine (default), sysid v3 multi-band, sysid v4 chirp |
| `--kp-pos`, `--kp-ori` | impedance gains; default `control.*` in `robot.yaml` |
| `--err-delta-pos`, `--err-delta-rot` | `osc_shm` error clamps; the chirp needs `0.15` / `0.80` |
| `--rate`, `--duration` | loop rate [Hz] (50) and run time [s] (4 / 12 / 8 by mode) |
| `--amp`, `--freq` · `--amp-x/y/z`, `--amp-yaw`, `--amp-roll` · `--f0`, `--f1`, `--amp-rx/ry/rz` | reference shape for sine · multiband · chirp |
| `--log run.csv [--sidecar run.json]` | write the CSV + metadata ([format](data_format.md)) |
| `--dry-run` | build the reference and print peak rates without a robot |

Modes:

| mode | reference | default duration | typical gains |
|---|---|---|---|
| `sine` | ±`--amp` (5 cm) z-sine at `--freq` (0.5 Hz) | 4 s | yaml defaults |
| `multiband` | SysID v3: two-band sinusoids on x/y/z + yaw/roll (`frankatwin.excitation.multiband`) | 12 s | `--kp-pos 200 --kp-ori 20` |
| `chirp` | SysID v4: 6-DOF linear chirp `--f0 0.1 → --f1 0.7` Hz, π/3 phase-staggered (`frankatwin.excitation.chirp`) | 8 s | `--kp-pos 500 --kp-ori 30 --err-delta-pos 0.15 --err-delta-rot 0.80` |

#### Example 3 · Run a policy closed-loop

Script: [`examples/policy_loop.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/policy_loop.py)

**What the robot does.** With the stand-in policy the EE rises 10 cm over 2 s,
descends 10 cm over the next 2 s, and repeats four times (16 s), then holds
where it ends. With your own policy it does whatever the actions command — bound
them with `--pos-scale` / `--rot-scale`, and keep the user stop in hand on the
first runs.

Continuous control driven by a policy at a fixed rate. Ships with a stand-in
policy that moves the EE up 10 cm and back down every 4 s for 16 s; swap in
your network.

```bash
python examples/policy_loop.py                          # demo policy, 10 Hz, 16 s
python examples/policy_loop.py --hz 20 --pos-scale 0.0025 --no-reset
```

| flag | meaning |
|---|---|
| `--hz`, `--duration` | policy rate [Hz] (10) and run time [s] (16) |
| `--pos-scale`, `--rot-scale` | action → Δpos [m/step] (0.005) and Δrot [rad/step] (0.02) |
| `--kp-pos`, `--kp-ori`, `--err-delta-pos`, `--err-delta-rot` | impedance gains (500 / 30) and clamps (0.15 / 0.80) |
| `--no-reset` | skip the initial `move_to` home |

The whole loop — the pattern every closed-loop rollout uses:

```python
def demo_policy(t, obs):                      # stand-in for a network; 6-D action in [-1, 1]
    a = np.zeros(6)                           # a[0:3] = Δxyz, a[3:6] = Δrot (axis-angle), base frame
    a[2] = 1.0 if t % 4.0 < 2.0 else -1.0     # up for 2 s, down for 2 s: 10 cm at 0.005 m/step, 10 Hz
    return a

with FrankaTwinClient(cfg) as robot:
    robot.move_to_q(cfg.robot.init_q)                         # 1. position control: reset to home
    robot.set_gains(kp_pos=500, kp_ori=30,                    # 2. impedance gains (Kd = 2*sqrt(Kp));
                    error_delta_pos=0.15, error_delta_rot=0.80)   #    clamp > one step, so it never engages
    s = robot.wait_for_state()
    t0 = time.monotonic()
    for k in range(int(16 * 10)):                             # 3. 16 s at 10 Hz
        obs = np.concatenate([s.q, s.dq, s.ee_pos, s.ee_quat, s.ee_linvel, s.ee_angvel])
        a = np.clip(demo_policy(k / 10, obs), -1, 1)
        pos  = s.ee_pos + 0.005 * a[:3]                       # 4. Δ on the measured pose (as in the sim task)
        quat = mul_wxyz(from_rotvec_wxyz(0.02 * a[3:]), s.ee_quat)
        robot.set_ee_target(pos, quat)                        # 5. non-blocking; osc_shm holds it at 1 kHz
        time.sleep(max(0.0, t0 + (k + 1) / 10 - time.monotonic()))   # 6. fixed-rate tick
        s = robot.get_state()                                 # 7. newest frame of the 100 Hz stream
```

Between two policy steps `osc_shm` applies `τ = Jᵀ[Kp e − Kd ẋ]` a hundred times
to the held target (zero-order hold); the IsaacLab replay reproduces exactly that
staircase, which is why the sysid transfers. `error_delta_pos` is set above the
largest single step so the controller never clips — the sim's task impedance has
no clip either. Targets are absolute poses in the base frame, quaternions
**wxyz**; gains and clamps persist across `move_to` and controller restarts.
Control law, safety chain and timing: [Architecture](architecture.md); every
client method and protocol: [Interfaces](interfaces.md); every config key:
[Configuration](configuration.md).

#### Example 4 · Open and close the gripper

Script: [`examples/gripper.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/gripper.py)

**What the robot does.** Only the Franka Hand moves; the arm keeps holding its
pose under impedance control. The hand is served on its own port (1338), so no
`osc_shm` stop/restart is involved — you can open and close while a policy loop
is running.

```bash
python examples/gripper.py --homing                      # once after power-up: calibrates the stroke
python examples/gripper.py --open                        # fully open (80 mm)
python examples/gripper.py --open --width 0.42           # 42 % of the stroke = 33.6 mm
python examples/gripper.py --close                       # grasp: squeeze at gripper.grasp_force (70 N)
python examples/gripper.py --close --force 30            # gentler hold
python examples/gripper.py --state                       # width / max_width / is_grasped / temperature
```

| flag | meaning |
|---|---|
| `--open [--width FRAC \| --width-m M]` | `Gripper::move` to a fraction of the stroke (default 1.0) or to metres |
| `--close [--force N] [--close-width M] [--eps M]` | `Gripper::grasp`; defaults from `robot.yaml → gripper` (70 N, −0.01 m, 0.08) |
| `--homing` / `--stop` / `--state` | calibrate / abort the motion in flight / read the state |
| `--speed` | finger speed [m/s]; default `gripper.move_speed` (0.1) / `gripper.grasp_speed` (0.5) |
| `--no-wait` | return once the daemon accepted the command |

**Closing is a grasp, not a width command.** libfranka's
`grasp(width, speed, force, eps)` drives the fingers *towards* `width` and
squeezes with `force` once they stall on something. The default width (−0.01 m,
past full closure) always reaches the object, so the resting width is set by the
object and `--force` is the knob for how hard it is held. `--close-width` only
stops the jaws early — above the object's width they halt before touching and
hold nothing. The returned `result` / `is_grasped` is "final width within
`epsilon` of the target"; with the default 0.08 m band every stall counts.

In a policy loop:

```python
robot.gripper_open()                              # blocks < 2 s; the arm holds meanwhile
...
out = robot.gripper_close(force=30)               # returns when the fingers stalled
held = out["state"]["width"]                      # the object's width, real-side twin of the sim finger joint
robot.gripper_open(0.06, wait=False)              # or fire-and-forget, then robot.gripper_wait()
```
