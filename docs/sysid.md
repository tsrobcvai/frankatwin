# System identification

FrankaTwin makes IsaacLab's Franka move like *your* real Franka robot under the
*same* impedance control. It replays real excitation runs in sim with identical
setpoints, gains and control law, and fits the sim's joint dynamics until the
trajectories match.

This page is the procedure to run it. For what the fit identifies, how the
optimizer works and how the excitations are designed, see
[SysID details](sysid_details.md).

## Workflow

### 1. Collect

<kbd>PC</kbd> with the daemon running on the NUC.

```bash
conda activate frankatwin
python examples/move_to.py
python examples/cart_impedance.py --mode chirp --rate 50 --kp-pos 500 --kp-ori 30 \
    --err-delta-pos 0.15 \
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
