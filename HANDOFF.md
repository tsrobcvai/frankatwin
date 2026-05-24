# panda_control — Project Handoff Context

This document is a self-contained context dump for any future AI agent (or
collaborator) working on this repo. Reading it should be enough to continue
the project without replaying the original design conversation.

---

## 1. What this project is

A from-scratch Franka FR3/Panda controller stack, built **bottom-up one small
step at a time**. Each step is a tiny program that exercises libfranka's
`robot.control(...)` at 1 kHz directly. The goal is a control surface that
matches the structure of the UR controller in
`omnireset/diffusion_policy/diffusion_policy/real_world/rtde_interpolation_controller.py`,
so that sim policies trained in IsaacLab (mirroring `omnireset/UWLab`'s
`RelCartesianOSCAction`) can be deployed on real Franka with the same control
law.

The user's broader goal is **sim2real for Franka**, replicating what the
UR setup already does end-to-end (sim → real with the same OSC math).

---

## 2. Why not the obvious alternatives

Three existing implementations were considered and explicitly rejected:

| Option | Where it lives | Why rejected |
|---|---|---|
| **deoxys** (`isaaclab_rollout/deoxys_control/deoxys`) | Treat NUC as a daemon, send commands via ZMQ + protobuf | Introduces trajectory interpolators (`LINEAR_POSE`, `MIN_JERK_POSE`, ...) that reshape policy actions. Breaks sim ↔ real 1:1 correspondence. Also: too much C++ to wade through, opaque debug surface. |
| **panda-py** | Python bindings to libfranka | Would mean zero C++. But: (a) single-maintainer project with firmware/version coupling risk, (b) Python in the 1 kHz callback path has GIL/GC issues, (c) less direct control over libfranka edges. Friend explicitly advised against. |
| **deoxys + new C++ controller** | Add a new controller into deoxys's multi-polymorphic stack | Too much C++ (controller .cpp + protobuf schema + node registration). User does not know C++ well. |

The chosen path: a **hybrid** of "one small C++ file at the bottom + N Python
files on top later". Step 1 onward is intentionally just a `franka::Robot`
+ `robot.control(callback)` standalone program with no ZMQ, no protobuf,
no daemon.

---

## 3. Reference codebases (read-only inputs)

These three repos drove the design. Do not modify them; they're context only.

```
/home/tao/Projects/omnireset/UWLab/
    source/uwlab_tasks/uwlab_tasks/manager_based/manipulation/omnireset/
        mdp/actions/task_space_actions.py         # RelCartesianOSCAction
        mdp/actions/actions_cfg.py
        config/ur5e_robotiq_2f85/actions.py       # gain configs
    # UR OSC in IsaacLab sim, PyTorch/batched. Control law:
    #   tau = J^T (Kp e + Kd e_dot), no inertial decoupling.

/home/tao/Projects/omnireset/diffusion_policy/
    diffusion_policy/real_world/
        rtde_interpolation_controller.py          # mp.Process, 500 Hz
        ur5e_kinematics.py                        # FK / Jacobian / OSC math
    # UR OSC on real robot, pure Python (uses rtde_control C++ underneath).
    # mp.Process + SharedMemoryQueue/RingBuffer pattern. NO interpolator
    # (deliberate for sim2real consistency). THIS IS THE STRUCTURE WE MIRROR.

/home/tao/Projects/isaaclab_rollout/deoxys_control/deoxys/
    franka-interface/src/controllers/*.cpp        # OSCImpedance, etc.
    franka-interface/src/franka_control_node.cpp  # ZMQ daemon
    deoxys/franka_interface/franka_interface.py   # Python client
    libfranka/                                    # vendored libfranka 0.20.0
    config/charmander.yml                         # robot/network config
    # The architecture we are NOT mirroring. But its built libfranka is
    # what we link against (see Section 6).
```

---

## 4. Project layout

```
panda_control/
├── CMakeLists.txt           # see Section 6 for the conda fix block
├── README.md                # operator-facing prereqs/build/run/validation
├── HANDOFF.md               # this file
├── .gitignore
├── src/
│   └── step1_joint_pd.cpp   # Step 1: Joint PD hold, ~400 lines, standalone
├── scripts/
│   └── run_step1.sh         # wrapper with safe defaults (KP=10, DURATION=3)
└── data/
    └── .gitkeep             # CSV logs land here
```

---

## 5. Roadmap (12-step plan, two phases)

The plan is split into two phases:

- **Phase A — controller increments (steps 1–9)**: each step is a standalone
  `src/stepN_*.cpp` running at 1 kHz, single-process, all params via CLI.
  Each step adds exactly one term or one feature to the control law, so
  any regression is bisectable to a single increment.
- **Phase B — productization (steps 10–12)**: wrap the final controller
  (step 9) with POSIX shared memory, build the Python
  `PandaController(mp.Process)` that mirrors `RTDEInterpolationController`,
  and wire up the sim2real eval harness.

| Step | Control law / change | Status | Key file |
|------|----------------------|--------|----------|
| **1** | Joint PD: `τ = Kp(q_des - q) - Kd·q̇`. Hold a fixed `q_des`. Validates 1 kHz loop, SCHED_FIFO RT priority, velocity noise floor. | **DONE & validated on real robot** (3002 ticks / 3.001 s, period 1.0 ms mean, dq RMS ≤ 1.1 mrad/s) | `src/step1_joint_pd.cpp` |
| **2** | Joint PD + gravity comp: `τ + g(q)`. libfranka's torque mode adds `g(q)` automatically → no new code, kept as a documented concept in the step 1 header and `README.md`. | **DONE (== step 1 semantics)** | (no new file) |
| 3 | + Coriolis comp: `τ + g(q) + C(q,q̇)·q̇`. Franka does **not** auto-compensate Coriolis; matters at non-trivial joint speeds. | **DONE (builds), not yet validated on real robot** | `src/step3_joint_pd_coriolis.cpp` |
| 4 | Joint trajectory tracking: same law as (3), sweep `q_des` smoothly. Find the Kp ceiling before shaking → that's the stable envelope. | **DONE (builds), not yet validated on real robot** | `src/step4_joint_traj.cpp` |
| 5 | Jacobian-transpose Cartesian PD (no Λ): `τ = Jᵀ(Kp·e + Kd·ė) + C(q,q̇)·q̇`, with `e = x_des - x`. Try the same gains in stretched-out vs. folded-up poses to see config-dependent stiffness directly. | **DONE (builds), not yet validated on real robot** | `src/step5_cart_pd.cpp` |
| 6 | Add inertial decoupling (full OSC): `τ = Jᵀ·Λ·(Kp·e + Kd·ė) + C(q,q̇)·q̇`, `Λ = (J·M⁻¹·Jᵀ + ε²·I)⁻¹`, `ε ≈ 1e-2`. Compare directly to (5) in the same poses; the difference is the value of decoupling. Push toward elbow extension to see Λ misbehave if `ε` is too small. | TODO | `src/step6_osc.cpp` |
| 7 | Split position and orientation: separate `Λ_p` (3×3), `Λ_o` (3×3), separate gains, separate errors. Validate orientation error standalone (axis-angle from `R_des · R_curᵀ` in the right frame). Most error-prone piece in the stack; worth isolating. | TODO | `src/step7_osc_pose.cpp` |
| 8 | Nullspace posture, kinematic projector: `τ += N · (Kp_null·(q₀ - q) - Kd_null·q̇)`, `N = I - J⁺·J`. With `Kd_null = 0` the elbow oscillates; set `Kd_null = 2·√Kp_null`. | TODO | `src/step8_osc_null.cpp` |
| 9 | Switch to dynamically consistent projector: `J̄ = M⁻¹·Jᵀ·Λ`, `N = I - Jᵀ·J̄ᵀ`. Posture torques now produce zero EE acceleration. Under aggressive posture gains, (8) disturbs the EE and (9) doesn't. | TODO | `src/step9_osc_null_dyn.cpp` |
| 10 | POSIX shared memory: step 9 binary becomes long-running; Python writes `target_pose / Kp / Kd` to shm. | TODO | `src/osc_shm.cpp` + `python/panda_control/shm_layout.py` |
| 11 | Python `PandaController(mp.Process)` mirroring `RTDEInterpolationController` (`SharedMemoryQueue` / `RingBuffer` pattern, no interpolator). | TODO | `python/panda_control/controller.py` |
| 12 | sim2real eval harness; plug in IsaacLab / ACT / diffusion policies. | TODO | `python/scripts/eval_real.py` |

**Important architectural note**: steps 1–9 are intentionally single-process
with all parameters via CLI. This is *not* a final shape; it's the
"learn libfranka first" stage. **Step 10 is the inflection point**: that's
when shared memory IPC enters and Python becomes a first-class citizen.

**Why step 2 has no new file**: libfranka's torque interface implicitly adds
**gravity `g(q)` and a friction-compensation term** to whatever we return
from the control callback. Verified in
`/home/nuc1/Projects/deoxys_control/deoxys/include/franka/robot.h:121`:

> "...joint-level torque commands **without gravity and friction** by
> providing callback functions."

So step 1's binary already runs as "Joint PD + gravity comp" semantically;
we simply never compute `g(q)` ourselves. Coriolis `C(q,q̇)·q̇` is **not**
auto-applied — that is exactly why step 3 needs a separate file: it has to
call `franka::Model::coriolis(robot_state)` and add the result by hand.
See the header of `src/step1_joint_pd.cpp` and the "Step 1" section of
`README.md` for the `τ_cmd` vs `τ_motor` breakdown.

---

## 6. Build environment & the conda fix

### Target machine: NUC (PREEMPT_RT, directly cabled to Franka)
- Kernel: `5.16.2-rt19 PREEMPT_RT` (confirmed)
- Project location on NUC: `/home/nuc1/Projects/panda_control/`
- Robot IP: `172.16.0.2` (Franka), NUC IP: `172.16.0.1`
- conda base is always activated in the user's NUC shell (`(base) nuc1@nuc1`)

### Why CMakeLists has a `CONDA_PREFIX` block

deoxys's `FrankaConfig.cmake` declares `fmt::fmt` as a PUBLIC link dependency.
`find_package(Franka)` therefore transitively does `find_package(fmt)`, which
resolves to conda's `libfmt.so.9.1.0` (built with GCC 11+, requires
`GLIBCXX_3.4.29`). The system `g++ 9.4.0` on Ubuntu 20.04 links against
`libstdc++.so.6` that only goes up to `GLIBCXX_3.4.28`, so the link fails:

```
/usr/bin/ld: /home/nuc1/anaconda3/lib/libfmt.so.9.1.0:
    undefined reference to `std::__throw_bad_array_new_length()@GLIBCXX_3.4.29'
```

Fix (mirrors `isaaclab_rollout/deoxys_control/deoxys/CMakeLists.txt:28-29`):
```cmake
if(DEFINED ENV{CONDA_PREFIX})
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -L$ENV{CONDA_PREFIX}/lib -Wl,-rpath,$ENV{CONDA_PREFIX}/lib")
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -L$ENV{CONDA_PREFIX}/lib -Wl,-rpath,$ENV{CONDA_PREFIX}/lib")
endif()
```
This pushes conda's lib path to the front of ld's search, so the implicit
`-lstdc++` finds conda's libstdc++ (`GLIBCXX_3.4.34`) instead of the system
one. **Do not remove this block** unless you also stop linking against
deoxys's libfranka build.

### How to build on the NUC
```bash
cd /home/nuc1/Projects/panda_control
rm -rf build                       # only if CMakeLists changed
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```
`find_package(Franka)` resolves automatically against deoxys's build tree
because `Franka_DIR` ends up at
`/home/nuc1/Projects/deoxys_control/deoxys/build/libfranka/`. If that ever
moves, set `-DCMAKE_PREFIX_PATH=<new-path>` at configure time.

### Mutual exclusion with deoxys
libfranka grants only one TCP session to the FCI port. **`deoxys`'s
`franka-interface` daemon must not be running** when `step1_joint_pd` (or any
later panda_control binary) starts. Verify with:
```bash
pgrep -a franka-interface || echo "ok"
```

---

## 7. Step 1 behavior (current shipped artifact)

`src/step1_joint_pd.cpp` is a single ~400-line file. The 1 kHz callback:

```cpp
double alpha = (args.ramp > 0.0) ? std::min(1.0, elapsed / args.ramp) : 1.0;
Eigen::Matrix<double, 7, 1> kp_eff = alpha * kp_target;
Eigen::Matrix<double, 7, 1> tau =
    kp_eff.cwiseProduct(args.q_des - q) - kd.cwiseProduct(dq);
tau = tau.cwiseMax(-TAU_LIMIT).cwiseMin(TAU_LIMIT);
```

Safety guards baked in:
- `Q_SAFETY_DELTA_RAD = 0.10` — refuses to start if `|q_init - q_des|_inf` exceeds it.
- Per-joint torque clamp `[87, 87, 87, 87, 12, 12, 12]` Nm.
- Kp ramp from 0 → target over `--ramp` seconds (default 1.5).
- SIGINT/SIGTERM → `franka::MotionFinished(zeros)`.

CLI is intentional — there is no runtime command channel yet (that arrives
in Step 10). The operator must:
1. In Franka Desk: unlock joints (blue LED) + Activate FCI.
2. Manually move robot to `q_des` (guiding mode).
3. `./scripts/run_step1.sh` (defaults: `KP=10, DURATION=3`).

The binary prints jitter stats and `dq` RMS per joint on exit, and writes a
per-tick CSV log.

---

## 8. Key decisions already made (do not re-litigate)

- **q_des source**: CLI argument; operator manually positions robot first.
  (Not "read q_init", not "use MotionGenerator to drive there".)
- **Step 1 form**: pure C++ single file, no Python, no shm.
- **Architecture trajectory**: stay single-process through step 9, add shm
  at step 10. POSIX `shm_open` (not ZMQ) when the time comes — same machine,
  microsecond-class latency.
- **No panda-py**, no PyBind11 embedding of Python into the 1 kHz callback.
- **No interpolators**. Sim2real consistency is the hill we're dying on.

---

## 9. Open decisions (defer until ready)

- **Where will the Python policy run in step 10+?**
  - Option A: same NUC (shm only). Best for sim2real determinism. Requires
    NUC to host PyTorch inference (typically lightweight — small policies).
  - Option B: separate PC (network IPC). Needed if a beefy GPU is required.
    Reintroduces deoxys-class IPC latency (1-5 ms).
  - Step 1 jitter / dq noise data is now in hand (period 1.0 ms mean,
    dq RMS ≤ 1.1 mrad/s) — the loop has the headroom for option A. Final
    decision still deferred until the policy stack is chosen.

- **Gripper control**: not in any current step. Added once step 9
  (full OSC + dynamically consistent nullspace) is stable. Will use
  `franka::Gripper` in a separate small program first, then integrated.

- **Logging format**: currently a flat CSV per tick. Acceptable for
  steps 1–9. May switch to HDF5 or NPZ in step 10+ when policy episodes
  need richer structured logs.

---

## 10. Suggested prompt for the next agent

> I'm continuing work on `/home/nuc1/Projects/panda_control/`. Please first
> read `HANDOFF.md` end-to-end so you have full context. Step 1
> (`src/step1_joint_pd.cpp`) is implemented, builds, and is **validated on
> the real robot** (period 1.0 ms mean, dq RMS ≤ 1.1 mrad/s on the NUC's
> PREEMPT_RT kernel). Steps 2 (= step 1 semantics, libfranka auto-adds
> `g(q)`) and onward are governed by the 12-step roadmap in §5; the next
> code increment is **step 3** (`src/step3_joint_pd_coriolis.cpp`, joint PD
> + explicit Coriolis term). The `CONDA_PREFIX` lib fix in `CMakeLists.txt`
> (including the direct `libboost_filesystem.so.1.82.0` link) — **DO NOT
> REMOVE** unless you also stop linking against deoxys's libfranka build.
>
> My next concrete request is: \<state it here>
>
> Conventions:
> - I prefer Chinese-language replies (per workspace rule).
> - Add new steps as separate `src/stepN_*.cpp` files; do not modify earlier
>   steps in place.
> - When proposing C++ changes, keep RT discipline: no heap allocation in
>   the 1 kHz callback, no logging that flushes per tick.
> - Mirror the structure / control law of
>   `omnireset/diffusion_policy/diffusion_policy/real_world/rtde_interpolation_controller.py`
>   and `omnireset/UWLab/.../mdp/actions/task_space_actions.py` whenever
>   adding new control logic. Sim ↔ real 1:1 correspondence is the goal.
> - Network/system constraints: NUC is `nuc1@nuc1`, conda base is always
>   active, libfranka is borrowed from
>   `/home/nuc1/Projects/deoxys_control/deoxys/build/libfranka/`, deoxys's
>   `franka-interface` daemon must be stopped before running our binaries.

---

## 11. Quick file index

- `CMakeLists.txt` — build, with mandatory `CONDA_PREFIX` lib fix.
- `src/step1_joint_pd.cpp` — Joint PD 1 kHz hold.
- `src/step4_joint_traj.cpp` — Joint trajectory tracking (single-joint sinusoid sweep).
- `src/step5_cart_pd.cpp` — Jacobian-transpose Cartesian PD (position-only, no Λ).
- `src/step5b_cart_pose.cpp` — Jacobian-transpose Cartesian PD (6D pose, no Λ).
- `scripts/run_step1.sh` — safe-default wrapper.
- `scripts/run_step4.sh` — safe-default wrapper for step 4 sweep tests.
- `scripts/run_step5.sh` — safe-default wrapper for step 5 Cartesian PD tests.
- `scripts/run_step5b.sh` — safe-default wrapper for step 5b 6D pose tests.
- `scripts/plot_step5.py` — offline plotting for Step 5/5b CSV logs.
- `README.md` — operator-facing prereqs / build / run / validation checklist.
- `HANDOFF.md` — this file.
- `data/` — CSV logs land here (gitignored except `.gitkeep`).
