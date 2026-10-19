# Validation record

## Joint impedance update: 2026-10-18

The force-gap workflow now plans bounded Panda joint references from the EE
sequence and uses `joint_pd_v1` in both domains. Real hardware was not connected.
The previous Cartesian measurement is retained below as a comparison.

The loaded 50-second run recorded 50,000 samples with physical finger contacts.
The full run and independent audit passed:

| Metric | Previous Cartesian run | Joint impedance run |
| --- | ---: | ---: |
| Maximum joint-1 excursion from reset | 125.14 deg | 8.516 deg |
| Maximum final joint deviation from reset | about 97 deg | 0.1874 deg |
| Poses with settled samples | 4/9 | 9/9 |
| Maximum EE position error | 7.05 mm | 4.426 mm |
| Maximum EE orientation error | 0.1756 rad | 0.01507 rad |

- Maximum joint tracking error: 0.008802 rad; all seven joints return close to reset.
- The planned maximum speed/acceleration was 0.4463 rad/s / 0.3605 rad/s².
- There were 11,706 settled samples; mean base-Z force was 3.041097 N, versus
  the nominal weight 3.041100 N.
- Static force RMSE per base axis: `[0.05132, 0.04673, 0.03742]` N.
- Dynamic force balance RMSE against block Newton-Euler support:
  `[0.12443, 0.07186, 0.06138]` N. These dynamic residuals are retained; better
  posture repeatability does not establish lower error for every force metric.
- Maximum physical grasp drift was 5.23 micrometers.
- Nominal planner FK and measured Isaac poses agreed within 0.631 micrometers
  and 1.71e-6 rad across 500 sampled configurations along the measured path.

A 3-second empty-hand run had settled force RMSE below 9.1e-7 N
on each base axis. It remains marked incomplete because it is a debug truncation.
An additional 2,999-step Isaac replay used a clearly marked
**synthetic input fixture derived from simulation**, not a real measurement.
Applied q/dq and EE reference arrays matched the fixture exactly at every tick.

Software validation: 121 tests passed. These cover planning between IK knots,
limit rejection, endpoint return, joint commands through loopback ZMQ, v5 ABI
sizes/offsets, full-rate applied references, and recovery on stale data, tracking
errors, mode changes and restarts. A compiled C++ test compared the torque math
used by `osc_shm` with the simulator for 2,800 inputs, including magnitude and
rate saturation; agreement was within 1e-12 absolute/relative tolerance.
`osc_shm.cpp` passed C++17 syntax compilation against libfranka 0.20 headers.
A linked NUC build and actual hardware tracking remain untested. The shared
memory ABI is now v5; update and rebuild the native controller and both Python
endpoints before real collection.

Local measurement and plots: `data/force_gap/sim_joint_preview_20261018/`.
`joint_control_audit.json` contains the detailed checks. The synchronized
dashboard is 1600 x 720, 20 fps, 50 seconds, 1,000 frames and 31.4 MB, with
fast-start MP4 metadata. Rendering reproduced measured EE
positions within 0.179 micrometers. `artifact_audit.json` pins the measurement
hash and video checks; the original October 17 video remains unchanged.
Generated measurements and audit files are local validation artifacts, excluded
from Git; follow the README to generate new runs on your PC.

## Historical Cartesian run: 2026-10-17

The v0.4 Beta feature was exercised in Isaac Lab 3 / PhysX with the bundled
Panda asset and a physically grasped 0.310 kg, 50 x 50 x 100 mm block.
No physical Franka was connected during this validation.

### Measurement run

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

A separate 3-second empty-hand check gave settled force RMSE below
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

Both output streams were inspected: there were headless display/NGX/OmniHub startup messages,
but no reported numerical failure, dropped-contact buffer, or overflow.

### Software checks

- 112 CPU tests passed, including independent wrench-transform fixtures,
  C++/NumPy ABI offsets, full-rate wrench transfer through loopback ZMQ,
  guided empty/loaded collection with the actual ring recorder, failure recovery,
  and comparison against a known synthetic bias.
- `osc_shm.cpp` passed C++17 syntax compilation against libfranka 0.20 headers
  and Eigen. A linked NUC build and physical hardware execution remain untested.
- The real collector dry-run produced 2,501 targets for the default 50-second
  sequence without a robot connection.
- A 3-second Isaac replay of an explicitly synthetic log fixture
  preserved all 3,000 applied Cartesian targets exactly and passed grasp/tracking
  checks. This validates the replay path, not physical real-robot behavior.
- Sphinx HTML documentation built successfully.

Real-versus-sim gap values are deliberately absent: they require a real run.
Software fixtures labeled as synthetic are used only for validation and are not
delivered as robot measurements.
