# Validation record: 2026-10-17

The v0.4 Beta feature was exercised in Isaac Lab 3 / PhysX with the bundled
Panda asset and a physically grasped 0.310 kg, 50 x 50 x 100 mm block.
No physical Franka was connected during this validation.

## Measurement run

- Slurm job: `4112613`, interactive partition, GPU node `neu320`.
- Duration: 50 seconds; 50,000 samples at a 1 ms physics step.
- All nine commanded poses visited; 1,000 video timestamps at 20 fps.
- Maximum block COM drift relative to EE: 4.63 micrometers.
- Maximum position / orientation target error: 7.05 mm / 0.176 rad.
- Tool-compensated base-Z force on settled samples: mean 3.04148 N;
  theoretical weight 3.04110 N.
- Force balance RMSE against independently computed block Newton-Euler support,
  in base axes: `[0.04740, 0.04239, 0.04469]` N.
- Torque balance RMSE about K, in base axes:
  `[0.00940, 0.01713, 0.00616]` N m.

Only pose indices 0--3 satisfy the conservative settled criterion; indices
4--8 retain joint motion above 0.04 rad/s during their holds. Their data remains
in dynamic plots but is excluded from static scores. The existing controller
has no null-space damping. This run does not establish nine stationary
calibration points. Noise and short transients are retained without smoothing.

A separate 3-second empty-hand check (`4112615`) gave settled force RMSE below
0.00002 N on every axis after tool compensation. Debug truncations are marked
incomplete even when their grasp/tracking checks pass.

The delivered scene is a rendering of recorded joint, finger and object poses,
using the original acquisition timestamps. Rendered EE positions agree with
recorded positions within 0.18 micrometers. The original physics video is retained
in the acquisition directory. The dashboard is 1600 x 720, 20 fps, 50 seconds;
the blue traces remain the original force data. Some headless RTX grain remains.

Local deliverables are under `data/force_gap/sim_preview_20261017/`, including
`dashboard.mp4`, the two frame-specific wrench figures, the static-pose figure,
raw/common samples, metadata, and `artifact_audit.json`. Generated measurement
files are intentionally excluded from Git.

The measurement NPZ SHA-256 is:

```text
13ca548750eae5987414eef5055d21417e6d7fd1d4c530b808ce3e84627174cc
```

Acquisition logs and exact launch commands are in
`/n/fs/rebar/isaaclab/repos/scratch/ft_gap_sim_full_20261017/`.
The independent monitor collected terminal status and stopped. Both output
streams were inspected: there were headless display/NGX/OmniHub startup messages,
but no reported numerical failure, dropped-contact buffer, or overflow.

## Software checks

- 112 CPU tests passed, including independent wrench-transform fixtures,
  C++/NumPy ABI offsets, full-rate wrench transfer through loopback ZMQ,
  guided empty/loaded collection with the actual ring recorder, failure recovery,
  and comparison against a known synthetic bias.
- `osc_shm.cpp` passed C++17 syntax compilation against libfranka 0.20 headers
  and Eigen. A linked NUC build and physical hardware execution remain untested.
- The real collector dry-run produced 2,501 targets for the default 50-second
  sequence without a robot connection.
- A 3-second Isaac replay of an explicitly synthetic log fixture (`4112677`)
  preserved all 3,000 applied Cartesian targets exactly and passed grasp/tracking
  checks. This validates the replay path, not physical real-robot behavior.
- Sphinx HTML documentation built successfully.

Real-versus-sim gap values are deliberately absent: they require a real run.
Software fixtures labeled as synthetic are used only for validation and are not
delivered as robot measurements.
