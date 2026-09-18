# SysID details

How the fit works and what it assumes: the procedure it follows, the 29
parameters it moves, the optimizer that moves them, and the excitations that
make them observable. The steps to actually run it are in
[System identification](sysid.md).

## Approach

We follow a procedure similar to [OmniReset](https://arxiv.org/abs/2603.15789) (UR7e),
itself based on PACE ([Bjelonic et al., 2025](https://arxiv.org/abs/2509.06342)):
record excitation runs on the real arm, replay them in sim under the same
controller, and fit friction, armature and motor delay with CMA-ES to minimize
the sim–real joint-trajectory error. One difference:

- **The two excitations swap roles.** Parameters come from a single multi-band
  sinusoidal run (v3, [Excitation design](#excitation-design)), not from chirps;
  the 6-DOF chirp (v4) that OmniReset fits on is held out as the test
  (joint-position MSE 4.8 × 10⁻⁴ rad²).

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

The control *law* is indeed the same on both sides — `τ = Jᵀ F`, pure task-space
PD (`control_mode="task_impedance"` in
`isaaclab_sysid/.../franka_sysid/control.py`). The Jacobian in it is not
obtained the same way:

- **Real** ([`src/osc_shm.cpp`](https://github.com/tsrobcvai/frankatwin/blob/v0.2_dev/src/osc_shm.cpp)):
  libfranka's analytic model, `model.zeroJacobian(Frame::kEndEffector)`, about
  the EE frame configured in Desk — the same frame the pose is read from.
- **Sim** (`franka_replay_env.py`): PhysX body Jacobians from
  `root_physx_view.get_jacobians()`, averaged over `panda_leftfinger` and
  `panda_rightfinger`, while the pose and velocity are read from a third body,
  `panda_fingertip_centered`.

So `J` differs between the two in both its source and its reference point.
CMA-ES absorbs that mismatch into the friction, armature and delay it fits — the
identified parameters therefore carry some geometric error that does not belong
to the joint dynamics at all.

## Optimizer

The fit runs [CMA-ES](https://github.com/CyberAgentAILab/cmaes) in
`isaaclab_sysid/scripts/tools/sysid_franka_osc.py`, with the 29 parameters
normalized to `[0, 1]` within the bounds above. Each generation evaluates
`--num_envs` candidates on every run at once: each run gets its own block of
`--num_envs` envs (the sim holds `num_envs × runs` envs), started from the run's
`q_init` and driven with the run's gains from its sidecar, and env `i` of every
block uses candidate `i`. The blocks replay on the same zero-order-hold time
base as `replay_python_csv_sim.py` — a 50-tick warmup, then each 50 Hz setpoint
held for its 20 × 1 ms ticks — and each candidate is scored at the CSV sample
instants with

```
loss = w_q · MSE(q_sim − q_real) + w_dq · MSE(q̇_sim − q̇_real) + w_x · MSE(x_sim − x_real)
       (w_q = 1.0, w_dq = 0.1, w_x = 0.01 by default)
```

summed over trajectories (weights: `--traj_weights`). 128 envs × 40 iterations
take about 2 h on one GPU; the cost of a 1 kHz tick barely depends on the env
count, so a larger population is nearly free. The best parameters so far are
saved every `--save_interval` generations to
`logs/sysid_franka/<timestamp>/checkpoint_<iter>.json`, and the final result to
`logs/sysid_franka/<timestamp>/sysid_best_params.json`.

`--eval_params <file>` skips the optimizer and rolls out one parameter set (a
`sysid_best_params.json` or a checkpoint) on the given runs: it prints the loss
per run and writes `eval_rollout_<run>.csv` and `eval_result.json`.

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
| Gains | `kp 200 / 20` | `kp 500 / 30` |

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
