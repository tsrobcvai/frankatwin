# Usage

:::{admonition} [TODO, checklist]
:class: warning

1. **Visualize the trajectories.** The three reference shapes in Example 2
   (`sine`, `multiband`, `chirp`) are described in prose only. Add a plot of each one so the shape,
   amplitude and duration can be seen before running it on the robot.
   `scripts/plot_ee_tracking.py` already plots actual-vs-target from a log, and
   `cart_impedance.py --dry-run` builds the reference without a robot, so the
   figures can be generated offline.
:::

## Basic control

Three steps: power on the arm, bring the controller up on the NUC, then drive
the arm from the PC. Every command below runs inside the `frankatwin` conda env
of that machine ([Installation](installation.md)).
Step 2 shows four usage examples (each script takes `--config robot.yaml`; the
two arm controllers they use are described in [Architecture](architecture.md)).

### Step 0 · Power on and prepare the arm

**Robot** — power on the arm and activate **FCI** mode in Desk.

### Step 1 · Start the daemon

<kbd>NUC</kbd>

```bash
conda activate frankatwin
python -m frankatwin.daemon -v
```

The daemon launches `osc_shm` — the arm now holds its current pose under impedance
control — and keeps it alive. Leave the terminal open; the banner should show
`RT = SCHED_FIFO`, `tau_rate = 800 Nm/s`, the payload and collision settings,
then `daemon ready`. All flags (`--config`, payload overrides), the banner line
by line, what it logs while running, how to stop it and when to restart it:
[Daemon](daemon.md).

### Step 2 · Drive the arm from the PC

<kbd>PC</kbd> — run the examples below from the root of the frankatwin repository:

```bash
conda activate frankatwin
cd frankatwin
```

#### Example 1 · Reset the arm

Script: [`examples/move_to.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/move_to.py)

**What the robot does.** The arm moves under stiff position control to a target
named one of two ways:

- **`--target-joints`** [default]{.badge-default} — seven joint angles.
  libfranka's `MotionGenerator` gives each joint a cubic ramp up, a
  constant-velocity cruise and a cubic ramp down, the seven synchronized to
  finish together, so the end effector sweeps an arc.
- **`--target-ee`** — a Cartesian pose. libfranka solves the IK and the end
  effector follows a straight 5th-order path.

:::{admonition} Safety
:class: danger

The arm does not yield on contact. Clear its path, keep the user stop in hand,
and use `--q-max-speed 0.1` the first time you try a new target.
:::

```bash
# Home: q = [0, -π/4, 0, -3π/4, 0, π/2, π/4] rad
python examples/move_to.py

# A joint configuration [rad], paced slowly
python examples/move_to.py --target-joints 0 -0.4 0 -2.0 0 1.6 0.785 --q-max-speed 0.5

# An end-effector pose: x y z [m] + quaternion wxyz, tool pointing down
python examples/move_to.py --target-ee 0.4 0.0 0.3  0 1 0 0  --q-max-speed 0.5
```

| flag | meaning |
|---|---|
| `--target-joints J1 … J7` | joint angles [rad] |
| `--target-ee x y z qw qx qy qz` | TCP position [m] and quaternion (wxyz) in the base frame; `0 1 0 0` = tool down |
| `--q-max-speed` | per-joint velocity cap [rad/s] for both modes, in (0, 1.25]; default `reset.q_max_speed` (0.5) |

> **`--q-max-speed` is approximate for `--target-ee`.** libfranka solves the IK
> internally, so the cap is only estimated — from the Jacobian at the start
> pose; for `--target-joints` it is exact.

#### Example 2 · Track scripted trajectories

Script: [`examples/cart_impedance.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/cart_impedance.py)

Streams a Cartesian reference to `osc_shm` at 50 Hz, then prints tracking error
and torque headroom.

**What the robot does.** Under impedance control the end effector tracks a
reference anchored at the pose it starts from, in one of three shapes:

- **`sine`** [default]{.badge-default} — ±5 cm along z at 0.5 Hz for 4 s,
  ending back at the start pose.
- **`multiband`** — x, y, z, yaw and roll, each driven at a low and a high
  frequency at once (0.15–1.1 Hz), for 12 s. Roughly ±12 cm in x and y, ±10 cm
  in z and ~0.5 rad of combined yaw/roll, peaking at 0.3 m/s.
- **`chirp`** — all six axes swept from `--f0` to `--f1` (0.1 → 0.7 Hz), for
  8 s. This is the sysid excitation.

:::{admonition} Safety
:class: danger

`multiband` and `chirp` are fast sysid motions, up to ±15 cm and ±0.5 rad.
Clear the space around the robot before running them.
:::

```bash
# Go home pose first
python examples/move_to.py

# Default: ±5 cm along z at 0.5 Hz for 4 s
python examples/cart_impedance.py

# Run the multiband trajectory
python examples/cart_impedance.py --mode multiband

# Sysid chirp with the gains and error clamps it needs, logged to CSV
python examples/cart_impedance.py --mode chirp \
    --kp-pos 500 --kp-ori 30 \
    --err-delta-pos 0.15 --err-delta-rot 0.80 \
    --log data/run.csv
```

| flag | meaning |
|---|---|
| `--mode` | `sine` [default]{.badge-default}, `multiband` or `chirp` |
| `--kp-pos`, `--kp-ori` | impedance gains; default from `robot.yaml` |
| `--err-delta-pos`, `--err-delta-rot` | error clamps [m] / [rad]; `chirp` needs `0.15` / `0.80` |
| `--log run.csv` | save a per-tick CSV and JSON sidecar ([format](data_format.md)) |
| `--dry-run` | build the reference and print peak rates, no robot |

Shape and duration flags are listed by `--help`; the `multiband` and `chirp`
designs are explained in [System identification](sysid.md#excitation-design).

#### Example 3 · Run a policy closed-loop

Script: [`examples/policy_loop.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/policy_loop.py)

Runs a policy at a fixed rate under impedance control. Replace the stand-in
`demo_policy` with your network.

**What the robot does.** The arm first resets to home (skip with `--no-reset`).
The stand-in policy then raises the end effector 10 cm over 2 s and lowers it
over the next 2 s, four times (16 s), and the arm holds where it ends.

:::{admonition} Safety
:class: danger

The reset is stiff position control, as in Example 1. A new policy moves the
arm wherever its actions point: keep `--pos-scale` / `--rot-scale` small and the
user stop in hand on the first runs.
:::

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

The whole loop:

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

Between policy steps `osc_shm` holds the last target at 1 kHz, the same
zero-order hold the IsaacLab replay uses. Keep `--err-delta-pos` above one step
so the clamp never engages, as in sim. Targets are absolute base-frame poses
with **wxyz** quaternions; gains and clamps persist across `move_to` and
restarts. More in
[Architecture](architecture.md), [Interfaces](interfaces.md) and
[Configuration](configuration.md).

#### Example 4 · Open and close the gripper

Script: [`examples/gripper.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/gripper.py)

**What the robot does.** Only the Franka Hand moves; the arm keeps holding its
pose under impedance control. The hand is served on its own port (1338), so no
`osc_shm` stop/restart is involved — you can open and close while a policy loop
is running.

:::{admonition} Safety
:class: danger

The fingers close at up to 0.5 m/s and squeeze with up to 70 N. Keep hands out
of the jaws; `--homing` sweeps the full stroke.
:::

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
| `--open [--width FRAC \| --width-m M]` | open to a fraction of the stroke (default fully open) or to a width [m] |
| `--close [--force N] [--close-width M] [--eps M]` | grasp; defaults 70 N, −0.01 m, 0.08 m |
| `--homing` / `--stop` / `--state` | calibrate / abort the motion / read the state |
| `--speed` | finger speed [m/s]; default 0.1 open, 0.5 close |
| `--no-wait` | return once the daemon accepts the command |

**Closing is a grasp, not a width command.** The fingers close until they stall
on the object, then squeeze at `--force`, so the object sets the held width and
`--force` sets how hard. `--close-width` only stops the fingers early; set wider
than the object, they hold nothing. `is_grasped` means the final width is within
`--eps` of the target; the default 0.08 m counts every stall.

In a policy loop:

```python
robot.gripper_open()                              # blocks < 2 s; the arm holds meanwhile
...
out = robot.gripper_close(force=30)               # returns when the fingers stalled
held = out["state"]["width"]                      # the object's width, real-side twin of the sim finger joint
robot.gripper_open(0.06, wait=False)              # or fire-and-forget, then robot.gripper_wait()
```
