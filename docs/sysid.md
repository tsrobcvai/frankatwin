# System identification

FrankaTwin makes IsaacLab's Franka move like *your* Franka under the *same*
controller, so what you tune or train in sim transfers. It replays real
excitation runs in sim with identical setpoints, gains and control law, and fits
the sim's joint dynamics until the trajectories match.

## Approach

We follow the procedure of [OmniReset](https://arxiv.org/abs/2603.15789) (UR7e),
itself based on PACE ([Bjelonic et al., 2025](https://arxiv.org/abs/2509.06342)):
record excitation runs on the real arm, replay them in sim under the same
controller, and fit friction, armature and motor delay with CMA-ES to minimize
the sim–real joint-trajectory error. Two differences:

- **Fit on one multiband run.** Parameters come from a single multi-band
  sinusoidal excitation (v3, [Excitation design](#excitation-design)), not from
  chirps.
- **Validate on a held-out chirp.** A 6-DOF chirp (v4), the excitation OmniReset
  fits on, is used only as the test: joint-position MSE 4.8 × 10⁻⁴ rad².

## What is identified

29 parameters, all per joint except the last:

| parameter | count | IsaacLab hook | bounds (default CLI) |
|---|---|---|---|
| armature (reflected rotor inertia) | 7 | `write_joint_armature_to_sim` | 0 – 0.5 kg·m² |
| μ_static | 7 | `write_joint_friction_coefficient_to_sim(static)` | 0 – 5 N·m |
| dynamic ratio (μ_dynamic = ratio · μ_static) | 7 | `… (dynamic)` | 0 – 1 |
| μ_viscous | 7 | `… (viscous)` | 0 – 5 N·m·s/rad |
| motor delay | 1 | `DelayedPDActuator` lag buffers (pos/vel/effort) | 0 – 4 ticks |

Everything else — link masses/inertias, kinematics, the controller — is taken as
known. Gravity is disabled on the sim articulation, mirroring libfranka's
gravity compensation on the real side.

## Optimizer

The fit runs [CMA-ES](https://github.com/CyberAgentAILab/cmaes) in
`isaaclab_sysid/scripts/tools/sysid_franka_osc.py`, with the 29 parameters
normalized to `[0, 1]` within the bounds above. Each generation replays the real
run at 1 kHz in `--num_envs` parallel envs, one candidate per env, and scores
each with

```
loss = w_q · MSE(q_sim − q_real) + w_dq · MSE(q̇_sim − q̇_real) + w_x · MSE(x_sim − x_real)
       (w_q = 1.0, w_dq = 0.1, w_x = 0.01 by default)
```

summed over trajectories (weights: `--traj_weights`). 128 envs × 40 iterations
take about 4 h on one GPU. The best parameters are saved every
`--save_interval` generations to
`logs/sysid_franka/<timestamp>/sysid_best_params.json`.

## Excitation design

A parameter is identifiable only if the recorded motion exercises it. Two
designs ship:

| | **v3 multiband** (`python scripts/gen_excitation_traj.py` / `python examples/cart_impedance.py --mode multiband`) | **v4 chirp** (`python scripts/gen_chirp_traj.py` / `python examples/cart_impedance.py --mode chirp`) |
|---|---|---|
| Spectrum | two stationary bands per axis (≈ 0.15–0.30 Hz + 0.7–1.1 Hz at 0.2× amplitude) | linear sweep 0.1 → 0.7 Hz on every axis |
| Active DOF | x, y, z + base-yaw + EE-roll | x, y, z, rx, ry, rz, π/3 phase-staggered |
| Amplitude | 10 / 10 / 8 cm; 0.25 / 0.20 rad | 10 / 10 / 15 cm; 0.50 / 0.25 / 0.50 rad |
| Envelope | symmetric 2 s half-cosine | 2 s up / 3 s down, linear |
| Duration | 12 s | 8 s |
| Gains | `kp 200 / 20` | `kp 500 / 30`, clamps `0.15 m / 0.80 rad` |

Design notes:

- **Excite rotation.** Translation alone barely moves j1 (base yaw) and j5
  (wrist roll), leaving their friction unobservable. Adding yaw and roll (v3)
  cut the sim–real RMS error from 88 to 39 mrad on j1 and from 29.5 to 10.1 mrad
  on j5.
- **The wrist caps the chirp at 0.7 Hz.** The 3 Hz sweep of the original UR5e
  design exceeds the 12 N·m limit of j5–j7 and trips the tracking abort. At
  0.7 Hz, peak end-effector speed stays near 0.46 m/s.
- **Replay at the logged rate.** Targets are sent at 50 Hz and held by the
  1 kHz controller; `replay_python_csv_sim.py` reproduces that hold, so logging
  at 50 Hz adds no sim–real mismatch. Never replay a 50 Hz log through a 1 kHz
  path.
- **Fit on several runs.** Use at least two runs with different frequency
  content, and hold one out.

## Workflow

### 1. Collect

<kbd>PC</kbd> with the daemon running on the NUC.

```bash
conda activate frankatwin
python examples/move_to.py
python examples/cart_impedance.py --mode chirp --rate 50 --kp-pos 500 --kp-ori 30 \
    --err-delta-pos 0.15 --err-delta-rot 0.80 \
    --log data/chirp_$(date +%Y%m%d_%H%M%S).csv
# optionally also a multiband run:
python examples/cart_impedance.py --mode multiband --kp-pos 200 --kp-ori 20 \
    --log data/multiband_$(date +%Y%m%d_%H%M%S).csv
```

Check the printed torque headroom (`max |tau_J| … (limits 87/…/12)`) and that
`abort.name == "none"` in the sidecar.

### 2. Fit

<kbd>SIM</kbd>

```bash
cd /path/to/IsaacLab
python scripts/tools/sysid_franka_osc.py --headless --num_envs 128 --max_iter 40 --sigma 0.3 \
    --real_csv /data/chirp.csv     --real_sidecar /data/chirp.json \
    --real_csv /data/multiband.csv --real_sidecar /data/multiband.json \
    --traj_weights 1.0,1.5
```

### 3. Validate

<kbd>SIM</kbd>

Replay a run — ideally one **not** used in the fit — with the fitted parameters:

```bash
python scripts/tools/apply_sysid_params.py \
    --best logs/sysid_franka/<ts>/sysid_best_params.json --invoke-replay \
    --real-csv /data/heldout.csv --real-sidecar /data/heldout.json --headless
# writes /data/heldout_sim_sysid.csv + .json
```

`--print-snippet` instead prints the actuator config overrides for your own task.

### 4. Compare

<kbd>PC</kbd>, in the `frankatwin` env (installed with `".[analysis]"`).

```bash
conda activate frankatwin
python scripts/compare_sim_real.py --real-csv /data/heldout.csv \
    --sim-csv /data/heldout_sim_sysid.csv --save
```

Produces position / orientation / per-joint overlays and prints RMS per axis and
joint.

## Limits and honest caveats

- The identified parameters are for the specific arm, payload and gains they were
  fitted under. Re-fit after changing the end-effector mass.
- The fit models joint-level dynamics under impedance control, so some Cartesian
  error remains. In our 6-DOF tracking tests, orientation matches to ≈ 1.5° RMS,
  but position is off by a few cm in some directions: the real arm moves about
  30 % further than the sim in x and y, and differs in z. The current parameters
  cannot represent such direction-dependent effects. Contributions welcome.
- `motor_delay_steps` is quantised to 1 ms ticks.

## Our results

Fit on one multiband run (v3), validated on a held-out chirp (v4):

| | baseline (PhysX defaults) | fitted |
|---|---:|---:|
| held-out chirp, joint-position MSE | — | **4.8 × 10⁻⁴ rad²** |
| held-out chirp, per-joint RMSE | — | 12–30 mrad (1.9–8.3 % of range) |
| training multiband, EE 3-D RMS | 41.1 mm | **8.4 mm** |
| training multiband, joint RMS | 983 mrad | **25.5 mrad** |

The held-out chirp (8 s, 6-DOF, never seen by the optimizer), replayed in
IsaacLab with the fitted parameters. Blue is the real arm, orange the twin,
dashed the commanded reference — the impedance controller lags the reference
identically on both sides, which is the point:

![Held-out chirp: real vs sim joint position and velocity, all seven joints](images/v3_sysid_v4chirp_joints.png)

![Held-out chirp: EE position x/y/z, target vs real vs sim](images/v3_sysid_v4chirp_position.png)

![Held-out chirp: EE orientation quaternion, target vs real vs sim](images/v3_sysid_v4chirp_orientation.png)

The remaining gap concentrates in j1 and j5 (base yaw, wrist roll) — the two
joints an EE-space excitation moves least, hence the rotation sweeps in the
v3 design. The tables below are the numbers behind the plots.

Per-joint RMS on the training multiband run [mrad]:

| | j1 | j2 | j3 | j4 | j5 | j6 | j7 |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 1718 | 539 | 1778 | 172 | 438 | 173 | 315 |
| translation-only fit | 88 | 25 | 89 | 38 | 30 | 53 | 64 |
| **+ rotation (v3)** | **39** | **13** | **38** | **16** | **10** | **19** | **27** |

Fitted parameters (`motor_delay_steps = 1`, i.e. one 1 ms tick):

| joint | armature [kg·m²] | μ_static [N·m] | μ_dynamic [N·m] | μ_viscous [N·m·s/rad] |
|---|---:|---:|---:|---:|
| j1 | 0.382 | 0.73 | 0.29 | 3.68 |
| j2 | 0.159 | 1.17 | 0.90 | 2.29 |
| j3 | 0.157 | 0.59 | 0.34 | 2.87 |
| j4 | 0.174 | 1.03 | 0.81 | 2.37 |
| j5 | 0.239 | 1.63 | 0.92 | 3.30 |
| j6 | 0.180 | 1.14 | 0.58 | 0.79 |
| j7 | 0.060 | 1.06 | 0.50 | 1.94 |

These are for *our* FR3 with a Franka Hand; friction varies unit to unit, so
run the fit on yours. Two things worth knowing about them:

- μ_static came *down* and μ_viscous *up* relative to translation-only fits —
  the extra joint motion lets CMA-ES separate stiction from damping instead of
  dumping everything into stiction.
- Armature mostly decreased (a more compliant arm model) once higher
  accelerations were present.
