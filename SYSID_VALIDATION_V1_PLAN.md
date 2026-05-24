# SysID Validation v1 — Handoff for NUC1

This document is a **self-contained handoff** so a fresh AI agent on NUC1 can
continue the Franka system-identification work without replaying our prior
chat. It describes:

1. What v1 sysid already produced (best params, scripts, dry-run validation).
2. The known design wart in the v1 pipeline (hard-coded `x_anchor`).
3. The recommended fix (修法 A) — implementation plan.
4. The exact commands to run on NUC1 to collect the new held-out trajectory.
5. The dev-machine commands to consume the new data (replay, ablation, report).

Read sections 1-2 first to understand the state, then jump to section 3-5 to
do work.

---

## 1. What v1 sysid already gave us

### 1.1 Files added in v1 (already on the dev machine)

| File | Role |
|---|---|
| `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/uw_insertion/franka_sysid_env_cfg.py` | Sysid env config: 1 kHz, `DelayedPDActuatorCfg(max_delay=4)` for arm, `ImplicitActuatorCfg` for gripper. |
| `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/uw_insertion/franka_sysid_env.py` | `Isaac-UW-Franka-Sysid-v0`: streams `(x_des, quat_des)` per tick to `uw_control.compute_dof_torque(control_mode="task_impedance")`. Exposes `set_targets()` and `step_replay(t_idx)`. |
| `IsaacLab/scripts/tools/sysid_franka_osc.py` | CMA-ES driver. 29-dim params (armature×7, mu_static×7, dynamic_ratio×7, mu_viscous×7, delay×1). Loss = `L_q + 0.1·L_dq + 0.01·L_x`. |
| `IsaacLab/scripts/tools/apply_sysid_params.py` | Injects best params into a fresh replay run (sub-process invocation). Supports `--invoke-replay` and `--print-snippet`. |
| `IsaacLab/scripts/tools/ablation_sysid_params.py` | Generates 5 derivative param JSONs (`all_on / armature_off / friction_off / viscous_off / delay_off`) and runs replay for each. |
| `panda_control/scripts/gen_excitation_traj.py` | 12 s multi-axis (2-band per axis) Cartesian target generator. Writes a step5b-compatible target CSV + sidecar. |
| `panda_control/src/step5c_excite.cpp` | Real-side task-impedance controller that **replays target CSV** at 1 kHz. Logs to step5b-compatible CSV/JSON. |
| `panda_control/scripts/record_excitation_traj.sh` | One-shot wrapper: gen target → run step5c_excite → sanity check. |
| `panda_control/scripts/report_sysid_v1.py` | Aggregates held-out + ablation into `summary.md` + `ablation_rms_z.png`. |

### 1.2 v1 best params

Reference path on dev machine:

```
IsaacLab/logs/sysid_franka/20260524_142300/sysid_best_params.json
```

| joint | armature [kg·m²] | mu_static [Nm] | mu_dynamic [Nm] | mu_viscous [Nm·s/rad] |
|---|---:|---:|---:|---:|
| j1 | 0.373 | 3.52 | 0.88 | 3.00 |
| j2 | 0.216 | 1.94 | 0.62 | 3.14 |
| j3 | 0.213 | 3.26 | 1.59 | 1.64 |
| j4 | 0.289 | 3.13 | 0.99 | 1.40 |
| j5 | 0.190 | 2.35 | 0.86 | 2.72 |
| j6 | 0.300 | 2.23 | 1.52 | 3.32 |
| j7 | 0.396 | 3.13 | 2.34 | 2.69 |
| global | `motor_delay_steps = 1` (≈ 1 ms @ 1 kHz) |

### 1.3 v1 dry-run results (held-out replaced by step5b itself, just to validate the pipeline)

Read `panda_control/data/sysid_v1_report/summary.md` for the auto-generated
table. Key numbers:

| metric | real | sim baseline | sim sysid |
|---|---:|---:|---:|
| err_pos_rms_xyz [mm] | [3.347, 1.062, 15.667] | [1.063, 0.128, 5.265] | [1.810, 0.139, 15.801] |
| err_pos_max_z [mm] | 24.508 | 9.310 | 24.722 |
| theta_max [mrad] | 31.770 | 3.186 | 32.165 |

Ablation z-RMS delta vs `all_on`:

| config | delta z RMS [mm] |
|---|---:|
| friction_off | **−6.32** (dominant) |
| viscous_off | −0.56 |
| armature_off | −0.02 |
| delay_off | 0.00 |

**Interpretation**: on this slow 0.25 Hz z-sin trajectory (step5b), friction is
the only term that is well-identified. Viscous, armature and delay barely
show up because the trajectory has almost no high-frequency content. That is
exactly why we need a richer held-out trajectory next.

---

## 2. Known design wart (do this on NUC1 first)

### Problem

`panda_control/scripts/gen_excitation_traj.py` and `panda_control/src/step5c_excite.cpp`
were wired together with a **hard-coded `x_anchor`** that came from step5b's
sidecar JSON:

- `gen_excitation_traj.py` reads `x_anchor` from `--base-sidecar` (default
  points at the step5b sidecar) and writes absolute Cartesian targets
  `x_des = x_anchor_step5b + ...` into the CSV.
- `step5c_excite.cpp` tracks those absolute targets. It only uses its own
  `readOnce()` pose for logging, not for the control law.

Consequence: the robot **must start at a pose whose EE position is close to
step5b's x_anchor** (within ~3 cm), otherwise either the 5 cm tracking abort
triggers, or the first 1.5 s of data is contaminated by a large transient.
This is awkward in practice — you should not have to manually teach the robot
back to step5b's pose every time.

### Decision (修法 A)

Add a tiny C++ binary `read_current_pose` that calls `robot.readOnce()` +
`model.pose(kEndEffector, state)` and writes a JSON in the **step5b sidecar
schema** (so `gen_excitation_traj.py` consumes it with `--base-sidecar` with
zero Python changes).

This decouples the trajectory generator from any frozen anchor. **Each time
you record, you read the robot's actual current pose, generate the CSV around
that pose, and run.**

---

## 3. NUC1 task list (修法 A implementation)

### 3.1 Add `panda_control/src/read_current_pose.cpp`

A ~80-line program. Skeleton (sufficient for an AI agent to fill in):

```cpp
// read_current_pose.cpp
// Calls robot.readOnce() + model.pose(kEndEffector), prints/writes JSON in
// the step5b sidecar schema so it can be fed into
// gen_excitation_traj.py --base-sidecar.

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {
// Reuse the same json_double / json_double_array / extract_pose_data helpers
// found in step5b_cart_pose.cpp / step5c_excite.cpp (copy verbatim).
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <robot_ip> [--out path/to/anchor.json]\n";
    return 1;
  }
  std::string robot_ip = argv[1];
  std::string out_path;
  for (int i = 2; i + 1 < argc; ++i) {
    std::string k = argv[i];
    if (k == "--out") out_path = argv[i + 1];
  }

  franka::Robot robot(robot_ip);
  franka::Model model = robot.loadModel();
  franka::RobotState s = robot.readOnce();
  // extract pose (column-major 4x4)
  // build q, x, quat_xyzw

  // Build JSON with these fields so gen_excitation_traj.py is happy
  // (see panda_control/scripts/gen_excitation_traj.py:_load_base_sidecar
  // for the schema it expects):
  //   {
  //     "q_init": [7],
  //     "x_anchor": [3],
  //     "q_anchor_xyzw": [4],
  //     "args": {"kp_pos": 200, "kp_ori": 20, "ramp": 1.5,
  //              "kd_pos": 28.28, "kd_ori": 8.94, "no_coriolis": false,
  //              "amp_ramp": 1.5}
  //   }

  // Print to stdout AND optionally to --out file. Always emit the JSON to
  // stdout so it can be piped/redirected.
  return 0;
}
```

### 3.2 Register the new target in `panda_control/CMakeLists.txt`

```cmake
add_executable(read_current_pose src/read_current_pose.cpp)
target_link_libraries(read_current_pose
    PRIVATE Franka::Franka Eigen3::Eigen Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(read_current_pose PRIVATE -Wl,--no-as-needed)
    target_link_libraries(read_current_pose PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(read_current_pose PRIVATE -O2)
```

### 3.3 Patch `panda_control/scripts/record_excitation_traj.sh`

Add a step 0 that reads pose, and feed that JSON into `gen_excitation_traj.py`
via `--base-sidecar`. The relevant diff region:

```bash
ANCHOR_JSON="${TMP_DIR}/${STEM}_anchor.json"

echo "[record_excitation] reading current robot pose ..."
"${ROOT_DIR}/build/read_current_pose" "${ROBOT_IP}" --out "${ANCHOR_JSON}"

echo "[record_excitation] generating excitation target around current anchor ..."
python "${SCRIPT_DIR}/gen_excitation_traj.py" \
  --base-sidecar "${ANCHOR_JSON}" \
  --out-csv "${TARGET_CSV}" \
  --out-sidecar "${TARGET_JSON}"
```

No change needed in `gen_excitation_traj.py` — it already reads everything
through `--base-sidecar`.

### 3.4 Build

```bash
cd ~/Projects/panda_control && mkdir -p build && cd build
cmake -DCMAKE_PREFIX_PATH="$HOME/Projects/isaaclab_rollout/deoxys_control/deoxys/build/libfranka" ..
cmake --build . --target read_current_pose step5c_excite -j 8
```

(Use whatever `CMAKE_PREFIX_PATH` you normally use for `step5b_cart_pose`;
the `read_current_pose` target reuses the exact same Franka + Eigen deps.)

---

## 4. NUC1: record the held-out trajectory

Once 3.1–3.4 are done, any-pose-start recording is a single command:

```bash
cd ~/Projects/panda_control
bash scripts/record_excitation_traj.sh 172.16.0.2
```

Expected outputs:

```
tmp/step5c_<ts>_anchor.json    # from read_current_pose
tmp/step5c_<ts>_target.csv     # from gen_excitation_traj.py
tmp/step5c_<ts>_target.json
data/step5c_<ts>.csv           # real-robot log, step5b schema
data/step5c_<ts>.json          # real-robot sidecar, step5b schema
```

The trajectory is 12 s @ 1 kHz with multi-axis 2-band sines:

```
x_des = x_anchor + 0.04 * (sin(2π·0.15·t) + 0.4·sin(2π·0.7·t))
y_des = y_anchor + 0.04 * (sin(2π·0.20·t + π/3) + 0.4·sin(2π·0.9·t))
z_des = z_anchor + 0.03 * (sin(2π·0.30·t + π/4) + 0.4·sin(2π·1.1·t))
quat_des = q_anchor (orientation held)
```

Peak linear speed ~0.14 m/s (well below the 0.3 m/s safety limit hard-coded
in `step5b_cart_pose.cpp`).

### Safety pre-flight (do once)

1. Robot is unlocked and FCI mode is active.
2. Free workspace of ~10 cm around current EE pose along all 3 axes.
3. First run: trim trajectory to 3 s before going full length, by editing
   `record_excitation_traj.sh` to pass `--duration 3.0` to `step5c_excite`.
   Once that succeeds, restore `--duration 12.0`.

### Sync back to dev machine

```bash
# from dev machine
rsync -av nuc1:~/Projects/panda_control/data/step5c_* ~/Projects/panda_control/data/
```

---

## 5. Dev machine: held-out + ablation + report

Substitute `step5c_<ts>` with the actual timestamp.

```bash
TS=20260524_HHMMSS
REAL_CSV=~/Projects/panda_control/data/step5c_${TS}.csv
REAL_JSON=~/Projects/panda_control/data/step5c_${TS}.json
BEST=~/Projects/IsaacLab/logs/sysid_franka/20260524_142300/sysid_best_params.json
PY=/home/tao/miniconda3/envs/isaaclab/bin/python

cd ~/Projects/IsaacLab

# (a) baseline sim replay (no sysid)
$PY scripts/tools/replay_real_step5b_sim.py \
  --real-csv $REAL_CSV \
  --real-sidecar $REAL_JSON \
  --out-csv     ~/Projects/panda_control/data/step5c_${TS}_sim.csv \
  --out-sidecar ~/Projects/panda_control/data/step5c_${TS}_sim.json \
  --headless

# (b) sysid sim replay
$PY scripts/tools/apply_sysid_params.py \
  --best $BEST --invoke-replay \
  --real-csv $REAL_CSV --real-sidecar $REAL_JSON \
  --out-csv     ~/Projects/panda_control/data/step5c_${TS}_sim_sysid.csv \
  --out-sidecar ~/Projects/panda_control/data/step5c_${TS}_sim_sysid.json \
  --headless

# (c) 5-config ablation
$PY scripts/tools/ablation_sysid_params.py \
  --best $BEST \
  --real-csv $REAL_CSV --real-sidecar $REAL_JSON \
  --out-dir ~/Projects/panda_control/data/step5c_${TS}_ablation \
  --headless

# (d) compare PNGs
cd ~/Projects/panda_control
$PY scripts/compare_sim_real_step5b.py \
  --real-csv $REAL_CSV \
  --sim-csv  data/step5c_${TS}_sim.csv \
  --out-dir  data/compare_sysid_step5c_${TS}_baseline --save
$PY scripts/compare_sim_real_step5b.py \
  --real-csv $REAL_CSV \
  --sim-csv  data/step5c_${TS}_sim_sysid.csv \
  --out-dir  data/compare_sysid_step5c_${TS}_sysid --save

# (e) final report
$PY scripts/report_sysid_v1.py \
  --real-csv         $REAL_CSV \
  --baseline-sim-csv data/step5c_${TS}_sim.csv \
  --sysid-sim-csv    data/step5c_${TS}_sim_sysid.csv \
  --ablation-json    data/step5c_${TS}_ablation/ablation_outputs.json \
  --out-dir          data/sysid_v1_report_step5c
```

Read `data/sysid_v1_report_step5c/summary.md` for the verdict.

---

## 6. Expected outcomes & success criteria

### Held-out generalisation

- `sysid` z-RMS on step5c within ±30% of real z-RMS → PASS.
- If FAIL (sim z-RMS far from real on step5c) → v2 should jointly fit
  step5b + step5c (multi-trajectory loss).

### Ablation contributions

On a higher-frequency trajectory like step5c, we **expect**:

- `friction_off` still dominant (similar to step5b dry-run, ~5-8 mm drop).
- `viscous_off` becomes non-trivial (~1-3 mm drop) thanks to the 0.7-1.1 Hz
  band.
- `armature_off` finally measurable (>0.1 mm drop) thanks to acceleration
  content.
- `delay_off` may become measurable (~0.1-0.5 mm drop).

If any group stays <1% impact → v2 can lock that group at the v1 best value
and drop the search dimension.

---

## 7. v2 candidates (after v1 validation closes)

Pick based on what v1 report tells us:

| Outcome of v1 | v2 action |
|---|---|
| All 4 groups have meaningful contribution | Re-run CMA-ES with `--num_envs 128 --max_iter 100` on combined step5b + step5c loss. Use new bounds informed by v1 best values (e.g. tighten armature to [0.1, 0.5]). |
| Armature/delay still <1% impact | Lock them at v1 values, reduce search to 14 dims, run CMA-ES on combined step5b + step5c for tighter friction/viscous fit. |
| Sysid sim z-RMS systematically too low | Excitation still under-frequenced → add chirp (0.05 → 2 Hz) and/or step-response trajectory. |
| Sysid sim z-RMS systematically too high | Likely overshoot of friction; reduce upper bound on `mu_static` from 5.0 to 3.5. |
| step5c PASS → policy transfer next | Bake v1 best params into `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/uw_insertion/franka_replay_env_cfg.py` (use `apply_sysid_params.py --print-snippet`). |

---

## 8. Reference paths cheatsheet

| Item | Path |
|---|---|
| Best params | `IsaacLab/logs/sysid_franka/20260524_142300/sysid_best_params.json` |
| v1 report (dry-run) | `panda_control/data/sysid_v1_report/summary.md` |
| Reference real CSV (step5b, used by v1 CMA-ES) | `panda_control/data/step5b_20260524_120834.csv` |
| Reference real sidecar | `panda_control/data/step5b_20260524_120834.json` |
| Sysid env cfg | `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/uw_insertion/franka_sysid_env_cfg.py` |
| Sysid env | `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/uw_insertion/franka_sysid_env.py` |
| Sysid driver | `IsaacLab/scripts/tools/sysid_franka_osc.py` |
| Apply / invoke replay | `IsaacLab/scripts/tools/apply_sysid_params.py` |
| Ablation runner | `IsaacLab/scripts/tools/ablation_sysid_params.py` |
| Real CSV/JSON schema | `panda_control/SIM2REAL_COMPARISON.md` |
| Real-side step5b controller | `panda_control/src/step5b_cart_pose.cpp` |
| Real-side step5c controller (replay-from-csv) | `panda_control/src/step5c_excite.cpp` |
| Target generator | `panda_control/scripts/gen_excitation_traj.py` |
| One-shot wrapper | `panda_control/scripts/record_excitation_traj.sh` |

---

## 9. TL;DR for the NUC1 agent

1. Implement `panda_control/src/read_current_pose.cpp` + register in CMake.
2. Patch `panda_control/scripts/record_excitation_traj.sh` to call it first
   and pass the resulting JSON via `--base-sidecar`.
3. Build, then `bash scripts/record_excitation_traj.sh 172.16.0.2`.
4. Sync `data/step5c_*.csv/json` back to dev.
5. Dev runs section 5 (a)-(e). Read `sysid_v1_report_step5c/summary.md`.

Nothing else in `IsaacLab/` or the sysid pipeline needs to change for v1
validation. v2 starts only after we see what step5c says.
