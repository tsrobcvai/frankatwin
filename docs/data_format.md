# Data format

Every real run and every sim replay produces a **CSV** (one row per control
tick) and a **sidecar JSON** (metadata, anchor, gains, summary). All tools in
`scripts/` and `isaaclab_sysid/scripts/tools/` consume these two files.

Quaternions in CSV/JSON are **xyzw** (the `RobotState` / shm side is wxyz).
Positions are in the robot base frame [m], angles [rad], torques [N·m].

## Real CSV (`python examples/cart_impedance.py --log`)

One row per Python tick (`--rate`, default 50 Hz). Row *k* holds the state
observed **just after** target *k* was sent; the target is zero-order-held until
row *k+1*.

| columns | meaning |
|---|---|
| `t_s` | nominal time `k / rate` |
| `period_ms` | measured loop period |
| `q1..q7`, `dq1..dq7` | joint position / velocity |
| `x_x, x_y, x_z` | EE position |
| `quat_x, quat_y, quat_z, quat_w` | EE orientation (xyzw) |
| `x_des_*`, `dx_des_*` | target position / velocity |
| `quat_des_*` | target orientation (xyzw) |
| `tau1..tau7` | commanded impedance torque (gravity excluded) |
| `tau_J1..tau_J7` | measured link-side torque (gravity included); NaN if the daemon predates shm v3 |

## Real sidecar (`<run>.json`, `schema_version: 2`)

```js
{
  "schema_version": 2,
  "controller": "python_v4_chirp_excitation",   // or python_multiband_excitation / python_sine
  "created_utc": "...", "csv_path": "...", "sidecar_path": "...",
  "control_rate_hz": 50.0, "duration_s": 8.0, "num_samples": 400,
  "amp_ramp_s": 0.0, "high_band_ratio": 0.0,   // multiband only
  "amplitude_m":   {"x": 0.10, "y": 0.10, "z": 0.15},
  "amplitude_rad": {"rx": 0.50, "ry": 0.25, "rz": 0.50},   // or {"yaw","roll"}
  "freq_set_hz": {...}, "phase_rad": {...}, "ori_freq_set_hz": {...}, "ori_phase_rad": {...},
  "peak_rates": {...},                          // pre-flight diagnostics
  "q_init": [7], "x_anchor": [3], "q_anchor_xyzw": [4],
  "args": {"kp_pos": 500, "kp_ori": 30, "kd_pos": 44.7, "kd_ori": 10.95,
           "duration": 8.0, "rate": 50.0, "mode": "chirp",
           "err_delta_pos_override": 0.15,
           "f0_hz": 0.1, "f1_hz": 0.7, "ramp_up_s": 2.0, "ramp_down_s": 3.0},
  "abort":   {"code": 0, "name": "none", "value": 0.0, "time_s": 0.0},
  "summary": {"ticks": 400,
              "err_pos_rms_xyz": [..], "err_pos_abs_max_xyz": [..], "err_pos_inf_max": ..,
              "err_ori_rms_rad": .., "err_ori_max_rad": ..,
              "tau_J_abs_max_nm": [7], "tau_J_limit_frac": [7], "tau_cmd_abs_max_nm": [7]}
}
```

The replay and sysid scripts read `q_init`, `x_anchor`, `q_anchor_xyzw` and
`args.kp_*`/`kd_*` (with `--gain-source sidecar`, the default) so the sim starts
from the same configuration under the same gains.

## Target CSV/sidecar (`scripts/gen_*_traj.py`)

The generators emit a 1 kHz reference (`t_s, x_des_*, dx_des_*, quat_des_*`) plus
a sidecar with the same anchor/gain fields and `"controller":
"multiband_excitation_target" | "multiband_pos_only_target" |
"v4_chirp_target"`. `python examples/cart_impedance.py` does not need them — it builds the
reference in-process — but they are useful for plotting the design and for
feeding other collectors.

## Sim CSV (`replay_python_csv_sim.py`)

Same time base as the real CSV (row *k* ↔ `t_s[k]`, state under target *k−1*), so
comparisons need no interpolation.

| columns | meaning |
|---|---|
| `t_s` | copied from the real CSV |
| `q1..q7`, `dq1..dq7` | joint state |
| `x_*`, `dx_*` | EE position / linear velocity |
| `quat_*` (xyzw), `wx, wy, wz` | EE orientation / angular velocity |
| `x_des_*`, `dx_des_*`, `quat_des_*` | target (copied) |
| `e_x, e_y, e_z, e_ox, e_oy, e_oz` | task-space error used by the controller |

The sim sidecar records the replay arguments, the applied sysid parameters (if
any) and the same `summary` block.

## `sysid_best_params.json`

```js
{
  "best_score": 0.002247,
  "best_params": [29 floats],                 // raw optimizer vector
  "best_params_decoded": {
    "armature":          [7],                 // kg·m²
    "mu_static":         [7],                 // N·m
    "dynamic_ratio":     [7],                 // mu_dynamic / mu_static ∈ [0, 1]
    "mu_dynamic":        [7],                 // N·m
    "mu_viscous":        [7],                 // N·m·s/rad
    "motor_delay_steps": 1                    // integer 1 ms ticks
  }
}
```

`apply_sysid_params.py --best … --print-snippet` prints the corresponding
`ArticulationCfg` actuator overrides for pasting into your own IsaacLab task.
