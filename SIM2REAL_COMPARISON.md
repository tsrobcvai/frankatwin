# Sim ↔ Real EE-Trajectory Comparison

This doc tells two future agents how to use the data we already record on the
real Franka to perform a sim-vs-real end-effector (EE) comparison.

- **Agent A — Sim Replay**: builds a script that drives a simulated Panda arm
  to follow the same target trajectory the real arm received, and writes its
  own per-tick log in the same CSV schema.
- **Agent B — Comparison & Plot**: builds a script that reads the real CSV
  + the sim CSV produced by Agent A, time-aligns them, computes per-axis
  position / orientation residuals, and saves comparison figures.

Reading this file plus the two example artifacts in §3 below is enough to
implement both scripts without further context.

---

## 1. Pipeline overview

```
┌──────────────────────────────┐     ┌──────────────────────────────┐
│ Real run                     │     │ Sim run (Agent A)            │
│ scripts/run_step5b.sh        │     │ same control law,            │
│ -> step5b_<ts>.csv  (real)   │ ─▶  │ same q_init / x_anchor,      │
│ -> step5b_<ts>.json (sidecar)│     │ same target stream           │
└──────────────────────────────┘     │ -> step5b_<ts>_sim.csv       │
                                     └──────────────┬───────────────┘
                                                    │
                                                    ▼
                              ┌──────────────────────────────────────┐
                              │ Comparison (Agent B)                 │
                              │ read both CSVs + the sidecar JSON,   │
                              │ time-align, sign-align quaternions,  │
                              │ compute residuals, save plots/report │
                              └──────────────────────────────────────┘
```

Why both sides log the same schema: the comparison script can then load
real and sim with one column map, regardless of which sim engine produced
the sim CSV.

---

## 2. Real-side artifacts (already produced)

Each real run produces two files in `panda_control/data/`:

- `step5b_<ts>.csv` — per-tick log at 1 kHz.
- `step5b_<ts>.json` — sidecar metadata (run config + initial state +
  summary).

### 2.1 CSV schema (66 columns, one row per 1 ms tick)

| Group | Columns | Meaning |
|---|---|---|
| time | `t_s`, `period_ms` | seconds since start; libfranka tick period in ms. |
| joints | `q1..q7`, `dq1..dq7` | joint positions (rad) and velocities (rad/s). |
| EE actual | `x_x,x_y,x_z` | EE position in **base frame** `panda_link0`, meters. |
| EE actual | `dx_x,dx_y,dx_z` | EE linear velocity in base frame, m/s. |
| EE actual | `quat_x,quat_y,quat_z,quat_w` | EE orientation as unit quaternion, **xyzw order**. |
| EE actual | `wx,wy,wz` | EE angular velocity in base frame, rad/s. |
| EE target | `x_des_x,x_des_y,x_des_z` | desired EE position from the trajectory generator. |
| EE target | `dx_des_x,dx_des_y,dx_des_z` | desired EE linear velocity. |
| EE target | `quat_des_x,quat_des_y,quat_des_z,quat_des_w` | desired EE orientation, xyzw. |
| Tracking error | `e_x,e_y,e_z` | `x_des - x` (m). |
| Tracking error | `e_ox,e_oy,e_oz` | `2 * vec(q_des * q^{-1})` after shortest-path sign flip (rad). |
| Torque (pre-clamp) | `tau_pd1..tau_pd7` | `J^T * f_task + c(q,dq)` (Nm). Real-only. |
| Torque (post-clamp) | `tau_cmd1..tau_cmd7` | clamped torque actually returned to libfranka (Nm). Real-only. |
| Coriolis | `c1..c7` | explicit Coriolis vector (Nm). 0 if `--no-coriolis`. Real-only. |

Conventions:

- Frame: **`panda_link0`** (Franka base). All `x_*`, `dx_*`, `wx/wy/wz`,
  `x_des_*`, `dx_des_*` are expressed in this frame.
- Quaternions: **xyzw** order (matches Eigen). They are normalized.
  Sign of the actual quaternion is *not* aligned to `quat_des` in the CSV,
  the comparison script must handle the q-vs--q ambiguity (see §5.2).
- Sample rate: 1000 Hz, target jitter < 0.1 ms std on PREEMPT_RT.
- Length: `t_s` starts at 0; row count ≈ `1 + duration * 1000`.

### 2.2 Sidecar JSON schema

```jsonc
{
  "schema_version": 1,
  "controller": "step5b_cart_pose",
  "started_utc": "2026-05-24T16:08:34Z",
  "csv_path": "/home/nuc1/Projects/panda_control/data/step5b_<ts>.csv",
  "sidecar_path": "/home/nuc1/Projects/panda_control/data/step5b_<ts>.json",
  "robot_ip": "172.16.0.2",
  "control_rate_hz": 1000,
  "frame": "panda_link0 (base)",
  "rt_priority": true,
  "args": {
    "kp_pos": 200.0, "kd_pos": 28.28, "kd_pos_user_set": false,
    "kp_ori": 20.0,  "kd_ori": 8.94,  "kd_ori_user_set": false,
    "axis": "z", "amp": 0.03, "freq": 0.25,
    "ori_axis": "z", "ori_amp_deg": 0.0, "ori_freq": 0.25, "ori_frame": "base",
    "duration": 8.0, "ramp": 1.5, "amp_ramp": 1.5,
    "no_coriolis": false, "print_err_every": 100
  },
  "q_init":        [q1, q2, q3, q4, q5, q6, q7],   // 7 joint angles, rad
  "x_anchor":      [x, y, z],                       // EE pos at startup, base frame, m
  "q_anchor_xyzw": [qx, qy, qz, qw],                // EE orientation at startup
  "abort": {
    "code": 0,                  // 0=none, 1=pos err, 2=joint limit, 3=ori err
    "name": "none",
    "joint": -1, "value": 0.0, "time_s": 0.0
  },
  "summary": {
    "ticks": 8001, "elapsed_s": 8.0,
    "period_ms_mean": 1.0, "period_ms_std": 0.0,
    "period_ms_min": 1.0,  "period_ms_max": 1.0,
    "dq_rms": [..7..], "c_rms": [..7..],
    "err_pos_rms_xyz": [..3..], "err_pos_abs_max_xyz": [..3..],
    "err_pos_inf_max": 0.0245,
    "err_ori_rms_xyz": [..3..], "err_ori_abs_max_xyz": [..3..],
    "err_ori_norm_max": 0.0317
  },
  "exception": null,
  "ended_normally": true
}
```

`q_init`, `x_anchor`, `q_anchor_xyzw` are the **single most important** fields
for sim replay: the sim arm must start from `q_init` so that EE-level
comparison isn't polluted by the 7-DOF redundancy / nullspace difference.

---

## 3. Concrete example pair (use these to test)

Real run on the actual Franka, kept under version control:

- CSV: `/home/nuc1/Projects/panda_control/data/step5b_20260524_120834.csv`
- JSON: `/home/nuc1/Projects/panda_control/data/step5b_20260524_120834.json`

Quick stats (from the sidecar):

- Trajectory: hold orientation at anchor, sweep position on **z** axis,
  amplitude **0.03 m**, frequency **0.25 Hz**, duration **8 s**, 1.5 s ramp.
- Gains: `kp_pos=200`, `kd_pos≈28.28`, `kp_ori=20`, `kd_ori≈8.94`,
  Coriolis on.
- Starting joint config (`q_init`):
  `[0.0767, -0.2437, 0.0869, -2.5058, 0.0065, 2.1595, 1.0495]` rad.
- Starting EE pose:
  `x_anchor = [0.4120, 0.0661, 0.2510]` m,
  `q_anchor_xyzw = [0.9972, -0.0539, -0.0489, 0.0186]`.
- Tracking on real: `err_pos_rms_xyz ≈ [3.3, 1.1, 15.7] mm`,
  `err_pos_inf_max ≈ 24.5 mm` (z, expected — that's the swept axis lag),
  `err_ori_norm_max ≈ 0.032 rad`.
- 8001 ticks, no abort, `ended_normally: true`.

Agents A and B should target this exact pair as their first end-to-end test.

---

## 4. Control law (the sim must match this)

Real-side controller is `src/step5b_cart_pose.cpp`. Reproduce these formulas
exactly in sim — any deviation will leak into the comparison and look like a
"sim2real gap" when it's actually a control-law gap.

```
J  = zeroJacobian(EE)       # 6x7, base frame
J_p = J[0:3, :]
J_o = J[3:6, :]

x   = FK_pos(q)             # base frame
R   = FK_rot(q)             # base frame
quat = quat_xyzw_from(R)    # normalized

v = J_p @ dq                # 3, EE linear velocity
w = J_o @ dq                # 3, EE angular velocity

# Orientation error (shortest path, body-rate-style 3-vector)
q_err = quat_des * inverse(quat)   # quaternion product
if q_err.w < 0: q_err = -q_err     # shortest path
e_o = 2 * q_err.vec                # = 2 * [q_err.x, q_err.y, q_err.z]

e_pos = x_des - x

f_task[0:3] = Kp_pos * e_pos - Kd_pos * v
f_task[3:6] = Kp_ori * e_o   - Kd_ori * w

c_vec = coriolis(q, dq)            # 7, omit (set to 0) if no_coriolis

tau_pd  = J.T @ f_task + c_vec     # 7
tau_cmd = clip(tau_pd, -TAU_LIMIT, TAU_LIMIT)
TAU_LIMIT = [87, 87, 87, 87, 12, 12, 12]   # Nm
```

Important: on the real robot libfranka **automatically adds gravity `g(q)`
and friction compensation** to whatever the callback returns. In sim you
must pick one of these, depending on what your sim integrator does:

- If sim's torque-control mode applies gravity automatically (most common
  in IsaacLab / MuJoCo with appropriate flags), do **not** add `g(q)` again.
- If sim does NOT auto-apply gravity, add `tau_g = M_g(q)` (gravity vector)
  to `tau_cmd` before stepping the integrator. **Document which path you
  took in the sim sidecar.**

Friction compensation is sim-dependent and the largest known mismatch
contributor — Agent B should expect a non-zero residual even with a perfect
pipeline.

### 4.1 Trajectory generator (only needed if you regenerate, not replay)

If Agent A wants to **regenerate** the target stream from `args` instead of
just replaying the CSV's `x_des_*` / `quat_des_*` columns:

```
omega    = 2*pi * freq
omega_o  = 2*pi * ori_freq
ori_rad  = ori_amp_deg * pi / 180

alpha_kp  = min(1, t / ramp)            # gain ramp
alpha_amp = min(1, t / amp_ramp)        # amplitude ramp
a_eff     = amp * alpha_amp
da_eff    = (amp / amp_ramp) if t < amp_ramp else 0.0

x_des = x_anchor.copy()
x_des[axis] += a_eff * sin(omega * t)
dx_des = zeros(3)
dx_des[axis] = da_eff * sin(omega * t) + a_eff * omega * cos(omega * t)

a_eff_o   = ori_rad * alpha_amp
theta_des = a_eff_o * sin(omega_o * t)
ax_unit   = unit_vector(ori_axis)        # [1,0,0] / [0,1,0] / [0,0,1]
q_delta   = quat_xyzw_from_axis_angle(ax_unit, theta_des)
quat_des  = (q_anchor * q_delta) if ori_frame == "ee" else (q_delta * q_anchor)
```

Note: `kp` is itself ramped by `alpha_kp` in the real controller, so the
*effective* gains in the first `ramp` seconds are smaller than the sidecar
`kp_*` values. Apply the same ramp in sim.

**Recommended for Agent A: replay the CSV columns directly** rather than
regenerating. That way any tiny floating-point drift in trig math doesn't
become a difference between sim and real.

---

## 5. Agent A — Sim Replay (specification)

### 5.1 Inputs

- `--real-csv`: path to a real `step5b_<ts>.csv`.
- `--real-sidecar`: path to its `step5b_<ts>.json`.
- `--out-csv`: where to write the sim log (default: alongside real with
  `_sim.csv` suffix, e.g. `step5b_<ts>_sim.csv`).
- `--out-sidecar`: optional, default mirrors `_sim.json`.

### 5.2 Setup steps

1. Load sidecar JSON. Read `q_init`, `x_anchor`, `q_anchor_xyzw`,
   `args.kp_pos / kd_pos / kp_ori / kd_ori`, `args.no_coriolis`,
   `args.ramp / amp_ramp`, `control_rate_hz`, `frame`.
2. Spawn the sim Panda arm (URDF/USD/MJCF, whichever your sim wants).
   Configure the integrator at `control_rate_hz` (1000 Hz) if possible;
   otherwise pick the highest-rate stable setting and **document the
   actual `control_rate_hz` you used in the sim sidecar.**
3. Drive the sim joints to `q_init` instantaneously (set `q = q_init`,
   `dq = 0`) before the control loop starts. **Do not run a planner/IK
   to get there** — the comparison only makes sense if both starts coincide.
4. Sanity check: compute FK at `q_init` and assert
   `||FK_pos - x_anchor|| < 1e-3` and quaternion alignment within
   `0.01 rad`. If this fails, the URDF / kinematic chain doesn't match
   the real robot — stop and report.

### 5.3 Per-tick loop

Pseudocode for tick `k`:

```
row = real_csv[k]              # one DataFrame row
t_s = row.t_s                  # seconds since start

# Target stream (replay mode — recommended)
x_des    = [row.x_des_x, row.x_des_y, row.x_des_z]
dx_des   = [row.dx_des_x, row.dx_des_y, row.dx_des_z]
quat_des = [row.quat_des_x, row.quat_des_y, row.quat_des_z, row.quat_des_w]

# Read sim state
q, dq = sim.get_joint_state()
J     = sim.zero_jacobian_ee(q)
x, R  = sim.fk_ee(q)
quat  = quat_xyzw_from(R)
v     = J[0:3] @ dq
w     = J[3:6] @ dq

# Same control law as real (§4). Apply gain ramp if t_s < ramp.
alpha_kp = min(1.0, t_s / args.ramp) if args.ramp > 0 else 1.0
kp_pos_eff = alpha_kp * args.kp_pos
kp_ori_eff = alpha_kp * args.kp_ori

e_pos = x_des - x
e_o   = quat_short_path_vec(quat_des, quat)

f_task = concat(kp_pos_eff * e_pos - args.kd_pos * v,
                kp_ori_eff * e_o   - args.kd_ori * w)

c_vec = sim.coriolis(q, dq) if not args.no_coriolis else zeros(7)

tau_pd  = J.T @ f_task + c_vec
tau_cmd = clip(tau_pd, -TAU_LIMIT, TAU_LIMIT)

# Add gravity ONLY if sim does not auto-apply it (see §4)
sim.set_joint_torque(tau_cmd)   # or tau_cmd + tau_g
sim.step()                      # advance one tick
```

If the sim integrator runs at < 1000 Hz, sub-sample the real rows by
nearest-neighbor or interpolate `(x_des, dx_des, quat_des)` onto the sim
grid. Keep `t_s` semantics identical so downstream alignment is trivial.

### 5.4 Sim CSV output schema

A subset of the real schema, sufficient for comparison. Required columns,
in this order:

```
t_s,
q1..q7, dq1..dq7,
x_x,x_y,x_z, dx_x,dx_y,dx_z,
quat_x,quat_y,quat_z,quat_w, wx,wy,wz,
x_des_x,x_des_y,x_des_z, dx_des_x,dx_des_y,dx_des_z,
quat_des_x,quat_des_y,quat_des_z,quat_des_w,
e_x,e_y,e_z, e_ox,e_oy,e_oz
```

Optional but useful: `tau_pd1..tau_pd7`, `tau_cmd1..tau_cmd7` (so torque
profiles can be compared too). `period_ms` and `c1..c7` are **not
required** for sim.

### 5.5 Sim sidecar (recommended)

Mirror the real sidecar shape; add a `source` block referencing the real
artifacts and a `sim` block describing the engine.

```jsonc
{
  "schema_version": 1,
  "controller": "step5b_cart_pose_sim_replay",
  "source": {
    "real_csv":      "/abs/path/to/step5b_<ts>.csv",
    "real_sidecar":  "/abs/path/to/step5b_<ts>.json"
  },
  "sim": {
    "engine":            "isaac_lab" | "mujoco" | "pinocchio+integrator",
    "robot_asset":       "<path or asset id>",
    "integrator_hz":     1000,
    "gravity_handling":  "engine_auto" | "added_in_controller",
    "friction_model":    "<short description>"
  },
  "args":          { ...same keys as real... },
  "q_init":        [...],
  "x_anchor":      [...],
  "q_anchor_xyzw": [...],
  "abort":         { "code": 0, "name": "none", ... },
  "summary":       { ...same shape as real... },
  "ended_normally": true
}
```

### 5.6 Failure modes Agent A should report

- FK at `q_init` doesn't match `x_anchor` / `q_anchor_xyzw`.
- Sim diverges (joint NaN, torque saturation for many ticks): write what
  you have, set `abort.code != 0`, set `ended_normally: false`.
- `control_rate_hz` mismatch >2× from real: log a clear warning in the
  sidecar.

---

## 6. Agent B — Comparison & Plot (specification)

### 6.1 Inputs

- `--real-csv`, `--real-sidecar`
- `--sim-csv`,  `--sim-sidecar` (sidecar optional but preferred)
- `--out-dir`: where PNGs and a JSON summary are written
  (default: `<real-csv-dir>/compare_<sim-stem>/`).

### 6.2 Time alignment

- Both CSVs use `t_s` starting at 0.
- If `len(real) == len(sim)` and `t_s` columns match within 1e-4 s,
  use index alignment directly.
- Otherwise interpolate the **sim** signals onto the **real** `t_s` grid:
  - linear interpolation for `x_*`, `dx_*`, `q*`, `dq*`, `e_*`.
  - quaternion: SLERP between sim samples (or normalize after linear
    interp + sign-align — SLERP is cleaner).

### 6.3 Quaternion sign alignment

Both real and sim store unit quaternions but `q` and `-q` represent the
same rotation. **Before any per-component subtraction or plot:**

```
for each i:
    if dot(q_real[i], q_sim[i]) < 0:
        q_sim[i] = -q_sim[i]
```

For numerical residuals use rotation-invariant forms:

- 3-vector residual: `e_compare = 2 * vec(q_sim * q_real.conjugate())`
  (apply shortest-path sign flip on the result quaternion).
- Scalar angle: `theta = 2 * acos(clip(|dot(q_real, q_sim)|, 0, 1))`.

### 6.4 Required metrics

Compute and emit (both per-axis and aggregate):

- `delta_pos = x_sim - x_real` per axis; RMS, max-abs, inf-norm over time.
- `||delta_pos||_2` time series, max and RMS.
- Orientation residual `e_compare` and `theta` time series, max and RMS.
- Per-side tracking: real already has `e_x..z, e_ox..z`; sim should too.
  Report RMS of each side so you can tell whether sim's *internal*
  tracking matches real's *internal* tracking.
- (Optional, if both have torque columns) `delta_tau_cmd = tau_cmd_sim
  - tau_cmd_real` per joint RMS — useful but secondary.

### 6.5 Required figures (PNG, 140 dpi)

Save under `<out-dir>/`:

1. `traj3d.png` — single 3D plot:
   `x_des` (gray dashed), `x_real` (blue solid), `x_sim` (orange dashed).
   Mark `x_anchor` with a scatter point.
2. `position_timeseries.png` — three stacked subplots (x, y, z), each with
   three lines: `*_des`, `*_real`, `*_sim`.
3. `position_residual.png` — `(x_sim - x_real)` per axis + `||·||_2`.
4. `orientation_residual.png` — both `theta(t)` (rad) and the 3-vector
   `e_compare` components. Y-axis label "rad".
5. `tracking_error_compare.png` — overlay `||e_pos||_real` and
   `||e_pos||_sim` on one plot, `||e_ori||_real` and `||e_ori||_sim`
   on another.
6. (Optional) `quaternion_timeseries.png` — sign-aligned q components
   (xyzw) for des/real/sim.

### 6.6 Summary report (machine-readable)

Write `<out-dir>/summary.json`:

```jsonc
{
  "real_csv": "...",
  "sim_csv":  "...",
  "n_samples_real": 8001,
  "n_samples_sim":  8001,
  "alignment": "index" | "interp_to_real",
  "delta_pos_rms_xyz_m":      [..3..],
  "delta_pos_max_abs_xyz_m":  [..3..],
  "delta_pos_norm_max_m":     0.0,
  "delta_pos_norm_rms_m":     0.0,
  "delta_ori_theta_max_rad":  0.0,
  "delta_ori_theta_rms_rad":  0.0,
  "real_internal_err_pos_rms_m":  [..3..],
  "sim_internal_err_pos_rms_m":   [..3..],
  "real_internal_err_ori_rms_rad":[..3..],
  "sim_internal_err_ori_rms_rad": [..3..]
}
```

Also print the same numbers to stdout.

### 6.7 Sanity checks Agent B must run

- Both sidecars (when present) have identical `args.kp_pos / kd_pos /
  kp_ori / kd_ori / axis / amp / freq / ori_*`. If not, refuse to compare
  and report the mismatch.
- Both sidecars have identical `q_init`, `x_anchor`, `q_anchor_xyzw`
  within tight tolerance (1e-4 rad / m). If not, refuse.
- Real `ended_normally == true` and `abort.code == 0`. If not, warn loudly
  and clip the sim trace to `min(real_end_t, sim_end_t)`.

---

## 7. Conventions cheat sheet

| Item | Value |
|---|---|
| Frame | `panda_link0` (Franka base) |
| Position units | meters |
| Quaternion order | **xyzw** (Eigen order), unit norm, no enforced sign |
| Angular velocity | base-frame, rad/s |
| Joint indexing | 1-based in CSV header (`q1..q7`), 0-based in code |
| Sample rate | 1000 Hz nominal, exact `t_s` in seconds is the source of truth |
| Torque limits | `[87, 87, 87, 87, 12, 12, 12]` Nm per joint |

---

## 8. Known mismatches (expect non-zero residual even with a perfect pipeline)

- **Friction** — real Franka has joint-level friction libfranka compensates
  for; sim friction model is engine-dependent. Will show up most in
  low-velocity regimes and at direction reversals.
- **Joint stiction near zero velocity** — same family as above.
- **Inertia / mass parameters** — Panda's URDF dynamic params are a public
  estimate; libfranka uses Franka's calibrated model. Expect a few percent
  inertia gap.
- **Sensor noise** — real `dq` has ≈1 mrad/s RMS noise floor; sim dq is
  noiseless. The PD `Kd_pos` term will therefore be slightly different
  on each side.
- **Communication latency** — sim has none; real has libfranka's
  one-tick command latency baked into the loop.

A "good" comparison after a successful sim2real bring-up typically shows:

- Position residual RMS in the millimeter range.
- Orientation residual RMS in the milliradian range.
- Internal tracking error (real vs sim's own `||e_pos||`) similar in
  shape, with sim usually a bit lower.

Larger residuals on the swept axis (here: z) than off-axes are expected —
that's the dynamics gap exposed by the trajectory excitation.
