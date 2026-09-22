# Data format

Every real run and every sim replay is a **CSV** (one row per control tick)
plus a **sidecar JSON**. Quaternions in CSV / JSON are **xyzw** (the
`RobotState` / shm side is wxyz); positions in the robot base frame [m], angles
[rad], torques [N·m].

## Real CSV (`cart_impedance.py --log`)

One row per Python tick (`--rate`, default 50 Hz); row *k* is the state just
after target *k* was sent, and the target is zero-order-held until row *k+1*.

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

(ring-log)=
## 1 kHz ring log (`cart_impedance.py --log-1khz`, `robot.log_start()`)

Every controller tick, recorded by the daemon in memory and written by the
client **on the PC** at the given path: `<stem>.csv`, `<stem>_targets.csv` and
the sidecar `<stem>.json`. Same columns as the real CSV, in the same order,
followed by:

| column | meaning |
|---|---|
| `seq` | `ShmStateFrame.seq`, the 1 kHz frame index; `t_s = (seq − seq_first) / 1000` |
| `target_idx` | row of `<stem>_targets.csv` whose setpoint was in force (−1 before the first) |

`x_des_*` / `quat_des_*` hold the setpoint in force at that tick (setpoint *k*
from row `seq = head_k + 1`, one tick of uncertainty); `dx_des_*` is 0.
`<stem>_targets.csv` has `target_idx, head, seq_effective, x_des_x, x_des_y,
x_des_z, quat_des_x, quat_des_y, quat_des_z, quat_des_w`.

The `log_stop` summary is stored as `ring_log` in the sidecar: `num_frames`,
`seq_first` / `seq_last`, `duration_s`, `num_targets`, `gaps` (frame ranges lost
to the 1024-frame ring), `dropped_frames`, `resets` (controller restarts),
`poll_hz`, `log_id`, and `save_error` if the fetch failed. A clean run has no
gaps and no resets. `scripts/shm_log.py` writes the same file without the
daemon; its `seq_effective` is late by up to one poll period.

## Sidecar (`<run>.json`, `schema_version: 2`)

```js
{
  "schema_version": 2,
  "controller": "python_v4_chirp_excitation",   // or python_multiband_excitation / python_sine
  "created_utc": "...", "csv_path": "...", "sidecar_path": "...",   // csv_path: the --log CSV, else the 1 kHz CSV
  "control_rate_hz": 50.0, "duration_s": 8.0, "num_samples": 400,
  "amp_ramp_s": 0.0,                            // multiband only
  "amplitude_m":   {"x": 0.10, "y": 0.10, "z": 0.15},
  "amplitude_rad": {"rx": 0.50, "ry": 0.25, "rz": 0.50},   // multiband: {"yaw","roll","rx","ry"}
  "freq_set_hz": {...}, "phase_rad": {...}, "ori_freq_set_hz": {...}, "ori_phase_rad": {...},
                                                // per axis. Frequencies: one per tone (multiband) or
                                                // [f0, f1] (chirp); phases: one per tone or [offset, 0]
  "peak_rates": {...},                          // pre-flight diagnostics
  "q_init": [7], "x_anchor": [3], "q_anchor_xyzw": [4],
  "args": {"kp_pos": 500, "kp_ori": 30, "kd_pos": 44.7, "kd_ori": 10.95,
           "duration": 8.0, "rate": 50.0, "mode": "chirp",
           "err_delta_pos_override": 0.15,
           "band": "low", "f0_hz": 0.1, "f1_hz": 0.7, "amp_taper_exp": 0.0,   // chirp only (high: 0.7, 3.0, 0.5)
           "ramp_up_s": 2.0, "ramp_down_s": 3.0},
           // multiband instead: "profile": "heldout", "ramp_down_s": 2.0 (fade-out),
           // "tone_rel_amp": {axis: [..]}, each tone's amplitude relative to amplitude_m / amplitude_rad
  "abort":   {"code": 0, "name": "none", "value": 0.0, "time_s": 0.0},
  "summary": {"ticks": 400,
              "err_pos_rms_xyz": [..], "err_pos_abs_max_xyz": [..], "err_pos_inf_max": ..,
              "err_ori_rms_rad": .., "err_ori_max_rad": ..,
              "tau_J_abs_max_nm": [7], "tau_J_limit_frac": [7], "tau_cmd_abs_max_nm": [7]},
  "ring_log": {"path": "...", "targets_path": "...",    // --log-1khz summary (the two CSVs on the PC), else null;
                                                        // paths null + "save_error" if the fetch failed
               "num_frames": 8000, "seq_start": .., "seq_first": .., "seq_last": .., "duration_s": 8.0,
               "num_targets": 400, "gaps": [], "dropped_frames": 0, "resets": 0, "poll_hz": 100.0,
               "log_id": 1}                             // counts the daemon's log_start calls
}
```

The replay and sysid scripts read `q_init`, `x_anchor`, `q_anchor_xyzw` and
`args.kp_*` / `kd_*` (`--gain-source sidecar`, the default).

## Target CSV / sidecar (`scripts/gen_*_traj.py`)

A 1 kHz reference, `t_s, x_des_*, dx_des_*, quat_des_*`, plus a sidecar with
the same anchor / gain fields and `"controller": "multiband_excitation_target"
| "multiband_pos_only_target" | "v4_chirp_excitation_target"`.

## Sim CSV (`replay_python_csv_sim.py`)

Same time base as the real CSV (row *k* ↔ `t_s[k]`, state under target *k−1*).

| columns | meaning |
|---|---|
| `t_s` | copied from the real CSV |
| `q1..q7`, `dq1..dq7` | joint state |
| `x_*`, `dx_*` | EE position / linear velocity |
| `quat_*` (xyzw), `wx, wy, wz` | EE orientation / angular velocity |
| `x_des_*`, `dx_des_*`, `quat_des_*` | target (copied) |
| `e_x, e_y, e_z, e_ox, e_oy, e_oz` | task-space error used by the controller |

The sim sidecar records the replay arguments, the applied sysid parameters and
the same `summary` block.

## Validation scores (`compare_<sim stem>/metrics.json`)

Written by `apply_sysid_params.py --invoke-replay` and `compare_sim_real.py
--save`. `ee` and `joints` are **sim against real**; `tracking` is each side
against the target. The sim is interpolated onto the real rows if its time base
differs.

```js
{
  "schema_version": 1, "created_utc": "...",
  "real_csv": "...", "sim_csv": "...", "real_sidecar": "...", "sim_sidecar": "...",
  "sysid_params": ".../sysid_best_params.json",      // via apply_sysid_params.py only
  "num_samples": 12000, "duration_s": 11.999,
  "ee": {
    "pos": {"rmse_m":    {"x": .., "y": .., "z": .., "3d": ..},   // x_sim − x_real; "3d" = RMS of the error's norm
            "max_abs_m": {"x": .., "y": .., "z": .., "3d": ..}},
    "ori": {"rmse_rad": .., "max_rad": ..}           // shortest angle between the sim and the real quaternion
  },
  "joints": {                                        // null if a CSV has no q1..q7
    "pos_rmse_rad": [7], "pos_max_abs_rad": [7],     // q_sim − q_real, per joint
    "pos_rmse_all_rad": ..,                          // over all seven joints at once
    "pos_mse_all_rad2": ..,                          // its square: what the fit minimises, unweighted
    "pos_rmse_pct_of_motion": [7],                   // 100 · rmse / (max − min of the real joint in this run)
    "real_motion_range_rad": [7],
    "vel_rmse_rad_s": [7], "vel_rmse_all_rad_s": ..  // only if both CSVs have dq1..dq7
  },
  "tracking": {"real": {"pos": {...}, "ori": {...}},  // x_des − x_real, angle(real, target): same fields as "ee"
               "sim":  {"pos": {...}, "ori": {...}}},
  "out_dir": "...", "metrics_path": "..."
}
```

## `sysid_best_params.json`

```js
{
  "best_score": 0.002247,                     // weighted total, Σ weight_b · loss_b over the runs
  "best_params_raw": [29 floats],             // optimizer vector in real units (checkpoints: "best_params")
  "best_params_decoded": {
    "armature":          [7],                 // kg·m²
    "mu_static":         [7],                 // N·m
    "dynamic_ratio":     [7],                 // mu_dynamic / mu_static ∈ [0, 1]
    "mu_dynamic":        [7],                 // N·m
    "mu_viscous":        [7],                 // N·m·s/rad
    "motor_delay_steps": 1                    // integer 1 ms ticks
  },
  "history": [{"iter": 1, "min": .., "mean": .., "best": .., "iter_s": .., "ms_per_tick": .., "eta_s": ..,
               "per_traj_min": [..],            // raw loss per run, best of the generation
               "best_candidate_weighted": [..]} // weight · loss per run for the generation's best candidate
              , ...],
  "bounds": [[lo, hi], ...],                  // 29 pairs, real units
  "args": {...},                              // the command line
  "source": {
    "trajectories": [{"real_csv": "...", "real_sidecar": "...", "num_samples": 8000,
                      "weight": 1.0,            // as used: 1.0, or the --traj_weights list
                      "gains": {"kp_pos": .., "kp_ori": .., "kd_pos": .., "kd_ori": ..}}, ...],
    "num_trajectories": 2, "time_base": "zoh_1khz_ticks_per_csv_row", "warmup_steps": 50
  }
}
```

`apply_sysid_params.py` reads `best_params_decoded` only; `--print-snippet`
prints those five arrays as a Python dict to paste into your own task config.
