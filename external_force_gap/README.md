# External force sim-real gap measurement

This v0.4 Beta workflow measures wrist wrenches while a Franka holds a known
rectangular payload and rotates through a shared pose sequence. It provides:

- A guided real-arm sequence: reset joints, verify the configured load, open the
  hand, place the block, grasp, verify width, move, and save full-rate data.
- An Isaac Lab 3 / PhysX sequence using the bundled Panda asset, physical finger
  contacts, and `JointWrenchSensor` on `force_sensor`.
- Raw telemetry, frame transforms, load metadata, tool-compensated simulated
  wrenches, gravity and rigid-body dynamics references, plots, and an optional
  synchronized video. A separate command compares actual real and sim bundles.

The default is a 50-second, nine-pose sequence with smooth 4-second transitions,
2-second holds, 50 Hz Cartesian targets and 1 kHz physics / real controller
logging. Rotations are rotation vectors relative to the anchor EE orientation.
Position is held at the anchor; the sequence ends at its initial orientation.
The arm can retain a different redundant joint posture, so reset before each run.

## What the signals mean

These are related measurements with different origins:

| Signal | Meaning | Axes | Torque reference |
| --- | --- | --- | --- |
| Franka `O_F_ext_hat_K` | Model-based external wrench estimate | Base O | K origin |
| Franka `K_F_ext_hat_K` | Same estimate | Stiffness K | K origin |
| `JointWrenchSensor` | Total incoming joint reaction, including downstream tool loads | Child-side joint frame S | Child-side joint anchor |
| Bundle `wrench_O_K` | Real: original API estimate. Sim: transformed joint wrench minus modeled tool support | Base O | K origin |
| Bundle `wrench_K_K` | Same canonical wrench rotated into K axes | K | K origin |

The canonical sign is **robot on payload**, following libfranka's documented
sign: forces applied by the robot to the environment are positive, and the
opposite direction is negative. A static, additional 0.310 kg payload therefore
requires **+3.0411 N along base Z**. With a downward-facing K frame the K-axis Z
reading is approximately -3.0411 N. The sign is checked against the gravity
reference; no automatic sign fit is performed.

`O_F_ext_hat_K` is an estimate derived from joint-torque residuals. Configured
robot, end-effector and load dynamics are removed by the robot model. It is not
the raw reading of a physical six-axis wrist transducer, and it can return six
zeros near a singularity. The collector saves both wrench representations,
`tau_ext_hat_filtered`, command success rate, and the configured mass/COM/inertia.
All-zero estimates are reported as candidates for investigation, not silently
discarded or classified as confirmed singularities.

The simulation uses a real sensor API reading, not the analytical gravity
formula as its measurement. FORGE uses the incoming joint-force path; the
`JointWrenchSensor` wrapper makes the child joint anchor/axes convention explicit.
The collector reads the joint's USD `localPos1` and `localRot1` and transforms:

```text
F_O = R_OS F_S
tau_K_O = R_OS tau_S + (p_S_O - p_K_O) cross F_O
O_T_K = O_T_EE EE_T_K
```

Expressing a wrench in base axes does **not** move its moment origin to the
base. Changing the moment origin requires the cross-product term above.
The raw Isaac Lab 3 pose API uses **xyzw**; FrankaTwin state/bundles use **wxyz**.
The simulator converts at every API boundary. Franka's flattened transform and
inertia arrays retain **column-major** order in the raw log; JSON transforms
are ordinary nested 4x4 matrices.

For each modeled body downstream of the wrist, the simulator computes support
from measured COM motion:

```text
F_support = m (a_COM - g)
tau_support_K = (p_COM - p_K) cross F_support + I_world alpha + omega cross (I_world omega)
```

The modeled tool includes the sensor link, hand, fingers and fingertip link.
Its support is subtracted from the joint reaction. The test block remains
unmodeled, as an additional external load. Accelerations are finite differences
of measured velocities; the raw signals are retained. The separate block
Newton-Euler reference checks balance during motion. Gravity-only comparisons
are scored on settled holds (after 0.7 seconds and with every arm joint below
0.04 rad/s). `completed` means the trajectory finished and the acquisition checks
passed; it does not imply every hold settled. `analysis.json` explicitly lists
poses without settled samples. The existing Cartesian controller has no
null-space damping, so redundant joint motion can continue during an EE hold;
such samples remain in dynamic traces and are excluded from static scores.
Franka's proprietary estimator, filtering, friction and model
errors are not reproduced by this compensation; they remain part of the gap.

References: [libfranka RobotState](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1RobotState.html),
[Isaac Lab joint wrench convention](https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/concepts/sensors/joint_wrench_sensor.html),
[FORGE](https://arxiv.org/abs/2408.04587).

## Payload and repeatable placement

Edit `config/force_gap.json` before collecting either domain. Weigh the printed
block: 310 g is an estimate, not an assertion about a particular 100% infill print.
Dimensions default to 50 x 50 x 100 mm. Its long axis is aligned with EE +Z.
The jaws close across its 50 mm width.

`ee_com_m` is the vector from **EE origin to block COM in EE axes**, not from the
wrist sensor. The default `[0, 0, 0.025]` means the block's proximal face is
25 mm behind the EE origin and the distal face is 75 mm ahead of it. Use a
placement jig or a clearly marked insertion-depth line to reproduce that pose.
Change the value to match the actual gripper/fingertip geometry and placement.
The script verifies width and `is_grasped`; these cannot measure insertion depth,
orientation, COM, or later small slips. The simulation also records actual
object pose relative to EE and reports its drift.

The controller's configured load must describe the **empty tool and any mounted
equipment**, excluding this test block. The collector displays and records the
actual API load state and asks the operator to verify it; it does not change
`setLoad`, tare the estimator, or remove controller gravity compensation. In the
existing daemon, `--load-mass 0` skips `setLoad`; it does not clear the Desk load.
An optional empty run provides a baseline and fixes the anchor, protocol, and
load configuration for the subsequent loaded run.

## Install and upgrade

From the repository root on the workstation and NUC:

```bash
pip install -e '.[analysis,test]'
```

This branch changes the shared-memory ABI from v3 to v4 (1024-byte frames).
Rebuild `osc_shm` using your normal libfranka toolchain and restart the daemon;
update the client and NUC Python package together. Older producers are rejected
by the collector instead of treating absent wrench data as zero.

```bash
cmake -S . -B build
cmake --build build -j
python -m frankatwin.daemon --config config/robot.yaml
```

The hardware software and motion execution still require commissioning on the
actual arm. Software tests use mock telemetry and do not certify physical motion.
The bundled simulation asset is a Panda. An FR3 or a custom tool requires a
matching asset; the replay checks initial FK and records actual motion mismatch.
The simulator uses Isaac Lab **3** APIs (`isaaclab_physx`, ProxyArray, xyzw), not
the older Isaac Lab 2 sysid environment. No copy into Isaac Lab is needed.

## Real collection

First inspect the sequence without connecting:

```bash
python scripts/force_gap_real.py --protocol config/force_gap.json \
    --out data/force_gap/preview --dry-run
```

With the daemon running, collect an empty baseline, then the held block:

```bash
python scripts/force_gap_real.py --config config/robot.yaml \
    --protocol config/force_gap.json --empty --out data/force_gap/real_empty

python scripts/force_gap_real.py --config config/robot.yaml \
    --protocol config/force_gap.json --baseline data/force_gap/real_empty \
    --out data/force_gap/real_loaded
```

The terminal requires explicit tokens for RESET, MODEL, OPEN, GRASP/EMPTY, and
RUN. Reset happens before placement. The loaded hand is never automatically
opened when acquisition ends or fails. The arm holds its final target; support
the block before opening it manually. On a runtime failure the collector tries
to hold the latest measured pose, stops/fetches the ring log, and saves the
exception and partial data. It rejects stale state, missing v4 telemetry,
excessive tracking error, invalid grasp width, changed load/frame settings,
dropped ring frames, and controller restarts. An unreachable daemon cannot
accept a software hold request; its existing local controller/reflex behavior
still governs the arm.

Actual producer timestamps and sequence IDs are preserved. The target events
are joined by sequence ID, with the existing logger's one-tick command timing
uncertainty. The 100 Hz ZMQ stream is only used for progress checks; the saved
measurement is the controller's full 1 kHz ring log.

## Simulation and first preview

Run from the FrankaTwin root with an Isaac Lab 3 Python environment:

```bash
/path/to/IsaacLab/isaaclab.sh -p "$PWD/scripts/force_gap_sim.py" \
    --protocol "$PWD/config/force_gap.json" \
    --out "$PWD/data/force_gap/sim_loaded" --headless --video

python scripts/force_gap_report.py analyze data/force_gap/sim_loaded --video
```

If a headless driver produces noisy RTX Real-Time 2.0 frames, render the recorded
states with the legacy RTX renderer and a wider camera. This does not rerun
dynamics or change any measurement. It checks rendered FK against the original
EE positions and records the measurement file hash in `render.json`:

```bash
/path/to/IsaacLab/isaaclab.sh -p "$PWD/scripts/force_gap_render.py" \
    "$PWD/data/force_gap/sim_loaded" --out "$PWD/data/force_gap/render" --headless \
    --kit_args="--/persistent/rtx/modes/rt/enabled=true"

python scripts/force_gap_report.py analyze data/force_gap/sim_loaded --video \
    --scene data/force_gap/render/scene.mp4
```

The replay uses the original frame timestamps and measured object pose, including
any recorded slip. The original acquisition video is retained. `dashboard.json`
identifies which scene was used; `--scene` requires a matching render manifest.

The hand closes with physical contacts; the payload is not welded or teleported
along the trajectory. Gravity acts on the robot and block. The arm uses the
same Cartesian impedance form and damping rule as `osc_shm`, with PhysX robot
gravity/coriolis compensation and an unmodeled grasped block. This does not
assume motor dynamics or friction are already identified.

For a simulator-only empty check add `--empty`. `--duration 3` is a debugging
truncation and marks the output incomplete; do not use it as a full experiment.
Output directories must be new to avoid overwriting an earlier measurement.

## Replay real commands and measure the gap

After collecting real data, replay its logged Cartesian targets, actual starting
joint positions, and recorded `F_T_EE` / `EE_T_K`:

```bash
/path/to/IsaacLab/isaaclab.sh -p "$PWD/scripts/force_gap_sim.py" \
    --real-run "$PWD/data/force_gap/real_loaded" \
    --out "$PWD/data/force_gap/sim_replay" --headless --video

python scripts/force_gap_report.py compare \
    --sim data/force_gap/sim_replay --real data/force_gap/real_loaded \
    --out data/force_gap/comparison
```

Comparison requires matching protocol and EE/K transforms, matching empty/loaded
conditions, and complete runs. It uses overlapping timestamps without
extrapolation, reports actual position/orientation/joint mismatch, and also
compares per-pose hold means and variability. There is no automatic sign,
scale, bias, or delay fit. `--time-offset` is an explicit, recorded diagnostic
offset added to real timestamps. `--allow-incomplete` is for inspecting failed
runs and preserves that qualification in the report.

The empty baseline is not automatically subtracted from the real estimate:
its measured bias is useful evidence, and unqualified subtraction would also
mix posture differences into the result. Keep and inspect both runs.

## Outputs

| File | Contents |
| --- | --- |
| `samples.npz` | Full arrays, including measured/target motion and raw/derived wrench channels |
| `samples.csv` | Common time, phase, wrench, gravity, and joint columns |
| `metadata.json`, `protocol.json` | Frames, load/asset provenance, sequence, completion and quality |
| `ring.csv`, `ring_targets.csv`, `wrench_raw.csv` | Real full-rate state/targets/API values; matrices flattened column-major |
| `session.json`, `events.json` | Real operator session result, exceptions, target phase events |
| `raw_partial.npz`, `failure.json` | Sim diagnostics if acquisition or validation fails |
| `wrench_O_K.png`, `wrench_K_K.png` | Six-channel time traces against independent references |
| `static_poses.png`, `analysis.json` | Hold means/std and gravity residual metrics |
| `scene.mp4`, `dashboard.mp4` | Rendered scene and synchronized K-frame force/torque plots |
| `comparison.json`, `gap_*`, `residual_*` | Real/sim gap statistics, overlays, difference plots and aligned CSVs |

Force statistics use N and torque statistics use N m. They are not combined in
an unscaled six-dimensional norm. Pose error bars show within-run variability,
not confidence intervals from independent repeated trials. A sim-only preview
is labeled as sim/theory validation; an actual sim-real gap requires real data.
One mass at one COM offset does not provide independent six-axis loading:
static force magnitude stays at `m g`, and torque is constrained by `r cross F`.
This workflow compares payload responses; it is not a complete six-axis sensor
calibration or a test of task-surface friction, impacts, or contact stiffness.

## Persistent monitoring on the cluster

`scripts/run_monitored.py` starts a persistent launcher and a separate monitor,
verifies the first status record, captures both streams, and records command,
PIDs, exit status, progress, possible errors and result paths. Use it for checks:

```bash
python scripts/run_monitored.py --run-dir "$PWD/data/force_gap/checks" \
    --interval 10 -- python -m pytest tests -q
```

For short GPU jobs, use `srun --partition=interactive` with a startup-inclusive
time limit up to two hours. Put the worker wrapper inside the allocation so it
records the job ID and compute-node process:

```bash
run="$PWD/data/force_gap/gpu_job"
python scripts/run_monitored.py --run-dir "$run" --interval 60 -- \
    srun --partition=interactive --gpus=1 --cpus-per-task=8 --mem=32G \
    --time=00:45:00 --job-name=ft-gap --unbuffered \
    python "$PWD/scripts/run_monitored.py" --run-dir "$run" --mode worker -- \
    /path/to/IsaacLab/isaaclab.sh -p "$PWD/scripts/force_gap_sim.py" \
    --out "$run/results" --headless --video
```

The local cluster's container invocation can be substituted after `--mode worker
--`. The monitor exits after terminal status is collected. It writes filesystem
status and does not itself deliver future chat messages. Inspect stdout and
stderr plus quality metrics: a process exit code alone is insufficient.
