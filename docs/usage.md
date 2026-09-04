# Usage

## Basic control

Two steps: bring the controller up on the NUC, then drive the arm from the PC.
Step 2 shows three usage examples (each script takes `--config robot.yaml`; the
two controllers they use are described in [Architecture](architecture.md)).

### Step 1 · Start the daemon

<kbd>NUC</kbd>

```bash
python -m frankatwin.daemon -v
```

It launches `osc_shm` — the arm now holds its current pose under impedance
control — and keeps it alive. Leave the terminal open; the banner should show
`RT = SCHED_FIFO`, `tau_rate = 800 Nm/s`, the payload and collision settings,
then `daemon ready`. All flags (`--config`, payload overrides), the banner line
by line, what it logs while running, how to stop it and when to restart it:
[Daemon](daemon.md).

### Step 2 · Drive the arm from the PC

<kbd>PC</kbd> — three usage examples, from a one-shot move to a closed-loop policy.

#### Example 1 · Reset the arm

Script: [`examples/move_to.py`](https://github.com/tsrobcvai/frankatwin/blob/v0.2/examples/move_to.py)

One-shot position control (`move_to`). Home by default; prints the pose
`osc_shm` holds afterwards.

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
