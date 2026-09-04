# System identification

The goal: make IsaacLab's Franka move like *your* Franka under the *same*
controller, so that anything you tune or train in sim transfers. FrankaTwin does
this by replaying real excitation runs in simulation — same setpoint staircase,
same gains, same control law — and fitting the sim's joint dynamics until the sim
joint trajectories land on the real ones.

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

`isaaclab_sysid/scripts/tools/sysid_franka_osc.py` runs
[CMA-ES](https://github.com/CyberAgentAILab/cmaes) in a `[0, 1]^29` latent space
mapped to the bounds above. Each generation evaluates `--num_envs` candidates in
parallel (one env per candidate) by replaying the real run at 1 kHz and scoring

```
loss = w_q · MSE(q_sim − q_real) + w_dq · MSE(q̇_sim − q̇_real) + w_x · MSE(x_sim − x_real)
       (w_q = 1.0, w_dq = 0.1, w_x = 0.01 by default)
```

summed over trajectories (optionally weighted with `--traj_weights`). With
128 envs and 40 iterations a fit takes ≈ 4 h on one GPU; the best parameters are
checkpointed every `--save_interval` generations to
`logs/sysid_franka/<timestamp>/sysid_best_params.json`.

## Excitation design

Identifiability is decided before you touch the optimizer. Two designs ship:

| | **v3 multiband** (`python scripts/gen_excitation_traj.py` / `python examples/cart_impedance.py --mode multiband`) | **v4 chirp** (`python scripts/gen_chirp_traj.py` / `python examples/cart_impedance.py --mode chirp`) |
|---|---|---|
| Spectrum | two stationary bands per axis (≈ 0.15–0.30 Hz + 0.7–1.1 Hz at 0.2× amplitude) | linear sweep 0.1 → 0.7 Hz on every axis |
| Active DOF | x, y, z + base-yaw + EE-roll | x, y, z, rx, ry, rz, π/3 phase-staggered |
| Amplitude | 10 / 10 / 8 cm; 0.25 / 0.20 rad | 10 / 10 / 15 cm; 0.50 / 0.25 / 0.50 rad |
| Envelope | symmetric 2 s half-cosine | 2 s up / 3 s down, linear |
| Duration | 12 s | 8 s |
| Gains | `kp 200 / 20` | `kp 500 / 30`, clamps `0.15 m / 0.80 rad` |

Design notes:

- **Rotation excitation is not optional.** A translation-only sweep produces a
  task wrench with no torque component, so j1 (base yaw) and j5 (wrist roll)
  barely move and their friction is unobservable. Adding yaw/roll sweeps (v3)
  cut j1 RMS from 88 → 39 mrad and j5 from 29.5 → 10.1 mrad versus the
  translation-only fit.
- **Chirp top frequency is bounded by the wrist.** The UR5e design this borrows
  from sweeps to 3 Hz; on the Franka that saturates the 12 N·m limit of j5–j7
  and trips the tracking abort. 0.7 Hz keeps peak `|ẋ|` ≈ 0.46 m/s and stays
  clear of the limits.
- **Log rate vs sim rate.** The Python loop commands at 50 Hz while the
  controller runs at 1 kHz. The replay reproduces exactly that zero-order-hold
  staircase (`replay_python_csv_sim.py`), so the 50 Hz sampling is not a source
  of sim–real mismatch. Do not use a 1 kHz replay path on a 50 Hz log.
- **Multiple trajectories beat one long one.** Fit on ≥ 2 runs with different
  spectral content and hold one out.

## Workflow

<kbd>NUC</kbd> = real-time PC on the robot, <kbd>PC</kbd> = your workstation, <kbd>SIM</kbd> = machine with IsaacLab (may be the PC).

### 0. One-time IsaacLab setup <kbd>SIM</kbd>

```bash
./isaaclab_sysid/install_into_isaaclab.sh /path/to/IsaacLab
conda activate <isaaclab env> && pip install cmaes
```

Installs `Isaac-FrankaTwin-Sysid-v0` / `Isaac-FrankaTwin-Replay-v0`
(`source/isaaclab_tasks/isaaclab_tasks/direct/franka_sysid/`, auto-registered),
`franka_mimic.usd` (Franka with a `panda_fingertip_centered` frame) and the three
scripts under `scripts/tools/`. Always launch the scripts from the IsaacLab root —
the task configs reference the USD relative to it.

### 1. Collect <kbd>PC</kbd> (daemon running on the NUC)

```bash
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

### 2. Fit <kbd>SIM</kbd>

```bash
cd /path/to/IsaacLab
python scripts/tools/sysid_franka_osc.py --headless --num_envs 128 --max_iter 40 --sigma 0.3 \
    --real_csv /data/chirp.csv     --real_sidecar /data/chirp.json \
    --real_csv /data/multiband.csv --real_sidecar /data/multiband.json \
    --traj_weights 1.0,1.5
```

### 3. Validate <kbd>SIM</kbd>

Replay a run — ideally one **not** used in the fit — with the fitted parameters:

```bash
python scripts/tools/apply_sysid_params.py \
    --best logs/sysid_franka/<ts>/sysid_best_params.json --invoke-replay \
    --real-csv /data/heldout.csv --real-sidecar /data/heldout.json --headless
# writes /data/heldout_sim_sysid.csv + .json
```

`--print-snippet` instead prints the actuator config overrides for your own task.

### 4. Compare <kbd>PC</kbd> (needs `pip install -e ".[analysis]"`)

```bash
python scripts/compare_sim_real.py --real-csv /data/heldout.csv \
    --sim-csv /data/heldout_sim_sysid.csv --save
```

Produces position / orientation / per-joint overlays and prints RMS per axis and
joint.

## Reference results

Fit on three multiband runs, validated on a held-out chirp:

| | baseline (PhysX defaults) | fitted |
|---|---:|---:|
| held-out chirp, joint-position MSE | — | **4.8 × 10⁻⁴ rad²** |
| held-out chirp, per-joint RMSE | — | 12–30 mrad (1.9–8.3 % of range) |
| training multiband, EE 3-D RMS | 41.1 mm | **8.4 mm** |
| training multiband, joint RMS | 983 mrad | **25.5 mrad** |

Per-joint RMS on the training multiband run [mrad]:

| | j1 | j2 | j3 | j4 | j5 | j6 | j7 |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 1718 | 539 | 1778 | 172 | 438 | 173 | 315 |
| translation-only fit | 88 | 25 | 89 | 38 | 30 | 53 | 64 |
| **+ rotation (v3)** | **39** | **13** | **38** | **16** | **10** | **19** | **27** |

Fitted values are in the README. Two things worth knowing about them:

- μ_static came *down* and μ_viscous *up* relative to translation-only fits —
  the extra joint motion lets CMA-ES separate stiction from damping instead of
  dumping everything into stiction.
- Armature mostly decreased (a more compliant arm model) once higher
  accelerations were present.

## Limits and honest caveats

- The identified parameters are for the specific arm, payload and gains they were
  fitted under. Re-fit after changing the end-effector mass.
- The fit captures joint-level dynamics under closed-loop impedance control. In
  our 6-DOF chase tests the *orientation* transfers to ≈ 1.5° RMS, while
  translational residuals of a few cm remain in some directions (real x/y move
  ~30 % further, z behaves differently) — anisotropic effects the current
  parameterisation cannot express. Contributions welcome.
- `motor_delay_steps` is quantised to 1 ms ticks.
