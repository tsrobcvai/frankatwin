# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.3.0] — 2026-09-21

### Added
- `heldout` multiband profile: the validation run for a chirp
  fit, same six DOFs and frequency range, different waveform — all tones at
  once instead of one frequency at a time (`MULTIBAND_PROFILES`,
  `multiband_tones()`, `--profile v3|heldout` on
  `examples/cart_impedance.py --mode multiband` and
  `scripts/gen_excitation_traj.py`). `v3` stays the default and reproduces the
  old reference bit for bit. With the tool pointing down, v3's yaw (about
  base z) and roll (about EE z) are the same rotation reversed, so its target
  never tilts the tool (`quat_des` z / w stay put) and its 0.2× upper tone
  holds 4 % of the reference's position power. `heldout` adds tilt about world
  x / y (`amp_rx` 0.20, `amp_ry` 0.15 rad, tool axis up to 20° off vertical;
  roll off), a third tone per axis at 1.25–2.0 Hz with every tone at
  min(1, 0.35/f) of the amplitude — constant peak reference velocity above
  0.35 Hz, the upper two tones holding 12–33 % of the position power — and a
  2 s fade-out, so the target ends at the start pose instead of 12 cm from
  it. Its reference stays inside the low-band chirp's (10 / 9 / 9 cm,
  0.47 rad, 0.45 m/s, 1.5 rad/s); the 1-DOF sizing model used for the high
  band (7.5 kg, calibrated on a high-band run) estimates 11 N of peak
  end-effector force per axis at kp 200 and 20 N at kp 500 (low band
  13 / 14 N, high band 33 N at kp 200). Estimates, translation
  only: the profile has not been run on the arm yet. `--amp-rx` / `--amp-ry`
  and `--amp-ramp-down` also work on the v3 table; `--high-band-ratio` is
  v3-only and rejected with `heldout`. Sidecars record `profile`,
  `ramp_down_s`, `tone_rel_amp` and one frequency / phase per tone.
  `docs/sysid.md` uses it as the validation run, at kp 500/30;
  `docs/sysid_details.md` has the design notes, the sizing table and the
  target plots.
- Two chirp bands for the fit, 8 s each (`CHIRP_BANDS`, `--band low|high` on
  `examples/cart_impedance.py --mode chirp` and `scripts/gen_chirp_traj.py`).
  `low` is the v4 production sweep, unchanged (0.1 → 0.7 Hz, constant
  amplitude, 2 s / 3 s ramps). `high` sweeps 0.7 → 3 Hz with every amplitude
  tapered as (f0/f)^0.5 and a 1 s fade-out
  (`build_chirp_trajectory(amp_taper_exp=0.5, ramp_down_s=1.0)`; the exponent
  is a knob: 0 = constant amplitude as in OmniReset's single 0.1 → 3 Hz chirp,
  1 = constant reference velocity, 2 = constant reference acceleration; the
  band now also sets the ramp defaults). Sized for the fit gains kp 200/20
  from a run on the arm at exponent 1: the arm followed 0.44 / 0.20 / 0.11 of
  the z reference at 1.4 / 2.0 / 2.6 Hz — an effective mass of ≈ 7.5 kg, a
  loop bandwidth of ≈ 0.8 Hz — leaving 34 / 11 / 2.8 mm of z motion and
  0.6–5 mrad per joint in the last window, at 13 / 23 / 5 % of the wrist's
  torque limit. The model calibrated on that run predicts 60 / 19 / 10 mm at
  ≈ 33 N for the new default (not yet run on the arm). It stops short of
  constant amplitude because every 50 Hz setpoint is a torque step that
  `osc_shm` ramps at 800 N·m/s and the sim does not: ≈ 1.6 N·m (2 ms) at 0.5,
  ≈ 3.1 N·m (4 ms, the whole delay search range) at 0. `cart_impedance.py`
  warns when the high band is run above kp 250 with an exponent below 2
  (≈ 84 N at 500/30). Analytic velocity includes the taper; sidecars record
  `band` / `amp_taper_exp`; `docs/sysid.md` collects both bands for the fit;
  `docs/sysid_details.md` shows both targets and the sizing table.
- 1 kHz ring log for sysid runs (`frankatwin.ring_log`): the daemon copies
  every controller tick out of the shm state ring and stamps each
  `set_ee_target` with the tick it took effect (`state_head` read right after
  the command write; `osc_shm` snapshots the command before publishing a
  tick's frame, so the target is in force from `head + 1`). Ops `log_start` /
  `log_stop` / `log_fetch`, `FrankaTwinClient.log_start()` / `log_stop()` /
  `log_save()` / `log_fetch()`, the same four on `LocalController`, and
  `examples/cart_impedance.py --log-1khz <path>`, which keeps the
  setpoint rate at `--rate` and records the run at 1 kHz alongside the 50 Hz
  log (summary stored as `ring_log` in the sidecar). The files land **on the
  PC**, not on the NUC: the daemon holds the session in memory, `log_stop`
  merges it, and the client pulls the rows in chunks — `log_fetch` replies with
  a JSON header plus a second ZMQ frame of raw float64, 49 per tick, 23.5 MB a
  minute — and writes the CSV and `<stem>_targets.csv` where the run was driven
  from. `cart_impedance.py` stops the log with the last setpoint and fetches
  once the arm is back at the anchor; the daemon keeps the last finished log
  until the next `log_start`, so a failed fetch can be repeated with
  `FrankaTwinClient.log_save(path)`. A `log_start` that still names a `path` on
  the daemon is refused, so a PC checkout from before this fails before the
  run instead of recording a log nobody collects. The CSV has the
  `--log` columns plus `seq` / `target_idx`, so `sysid_franka_osc.py` and
  `replay_python_csv_sim.py` read it unchanged, one tick per row. Motivation:
  a 50 Hz row's state is the newest frame of the 100 Hz stream, 0–10 ms older
  than its nominal `t_s` — more than the 0–4 ms motor delay the fit looks for.
  `scripts/shm_log.py` dumps the ring on the NUC without the daemon (setpoints
  at poll resolution). `--log-1khz` is enough on its own for a sysid run:
  the sidecar is written next to the CSV as `<stem>.json` (also when `--log` /
  `--sidecar` put one elsewhere), both falling back to the working directory
  if the write fails; a path whose directory cannot be created stops the
  script before it connects, and `--log` / `--log-1khz` naming the same file
  is refused. `--log` (the 50 Hz CSV) is optional.
  `tests/test_cart_impedance_logging.py` runs `main()`
  against a stand-in client. `tests/test_ring_log.py` drives an in-memory segment
  with a stand-in producer: lossless drain across ring wraparound, gap and
  restart accounting, tick-exact merge, a sim-readable CSV, and a log moved
  from the daemon's REP loop to a `FrankaTwinClient` over loopback ZMQ, written
  byte for byte as a recorder with a path writes it.
- `set_ee_target` (client, daemon op, `LocalController`) now returns the
  `state_head` seen right after the write (`0` from an older daemon).
- Franka Hand support: `gripper_cmd` C++ binary (libfranka `Gripper`, own
  connection on port 1338 — the arm controller keeps running), daemon ops
  `gripper_homing` / `gripper_move` / `gripper_grasp` / `gripper_stop` /
  `gripper_state` (asynchronous, polled), `FrankaTwinClient.gripper_open()` /
  `gripper_close()` / `gripper_homing()` / `gripper_stop()` / `gripper_state()`,
  `examples/gripper.py` (same `--open --width FRAC` / `--close --force N`
  semantics as the deoxys-based `control_gripper.py`), and real
  `robot.yaml → gripper:` keys (speeds, grasp force / width, epsilons).
  `doctor` checks the binary and the gripper port.
- Per-robot FCI lock (`src/fci_lock.h`): `osc_shm` and `move_to` take an
  advisory `flock` on `/tmp/frankatwin-fci-<ip>.lock` before connecting, so a
  second controlling session exits 5 with the holder's pid instead of dying
  inside libfranka on `Set Joint Impedance command rejected: command not
  possible in the current mode ("Move")`. Held on an open fd, so the kernel
  releases it even on SIGKILL — no stale locks. Read-only helpers and
  `gripper_cmd` take no lock and still run alongside the daemon. `doctor`
  reports the lock state as `fci lock`.
- `sysid_franka_osc.py --eval_params <file>`: roll out one fixed parameter set
  (a `sysid_best_params.json` or checkpoint) without CMA-ES; prints the loss per
  trajectory and writes `eval_rollout_<run>.csv` + `eval_result.json`.

### Changed
- Docs: the sysid procedure homes the arm (`examples/move_to.py`) before every
  collection instead of once per session (`docs/sysid.md`, `docs/usage.md`,
  the `cart_impedance.py` prerequisites). A run is anchored at the pose it
  starts from and does not return there: `osc_shm`'s torque law
  (`Jᵀ·F_task + c`) has no posture term, and in the three runs of 2026-09-21
  joint 1 ended 0.29 / 0.09 / 0.33 rad from its start (low band, high band,
  heldout) with the end effector 8–14 mm off.
- Docs: the fit uses the low chirp band alone; the high band is an optional
  second held-out run (`chirp_high_heldout.csv`), scored like the multiband.
  `docs/sysid.md` fits with `--num_envs 128` (about 1 h on an RTX 5090) and
  weights several runs with an explicit `--traj_weights 1.0,1.0`.
  `sysid_franka_osc.py --traj_weights` takes that list only (validated: one
  per run, all > 0); the fit prints, per iteration, what each run adds to the
  best candidate's total (`w*L of best`). The `auto` mode,
  `scripts/tools/traj_weights.py` and the `signal_power` / `traj_weights`
  fields of `sysid_best_params.json` are gone. The fit script needs Isaac Sim:
  it compiles, not run.
- Sysid validation is one command. `apply_sysid_params.py --invoke-replay`
  now runs the comparison right after the replay: the sim CSV, the overlay
  figures and a new `metrics.json` — sim against real, EE position RMSE per
  axis and 3-D, EE orientation RMSE, per-joint q / dq RMSE, the joint-position
  MSE, per-joint RMSE in percent of the real joint's motion, plus each side's
  tracking error against the target — all land next to the real CSV
  (`--compare-dir`, `--no-compare`, `--show`). `compare_sim_real.py` gained
  `compare()` / `compute_metrics()` and writes the same `metrics.json` when run
  alone; it used to print EE errors only, although the docs promised per-joint
  RMS. `docs/sysid.md` folds the old steps 3 and 4 into one.
  `tests/test_compare_sim_real.py` checks every score against closed-form
  offsets and runs the one-command path with a stand-in replay script (the
  real one needs Isaac Sim, so that half is untested here).
- Docs, caught by building the site and checking every anchor: all six links
  to the ring-log section of `docs/data_format.md` pointed at
  `#1-khz-ring-log`, an id that never existed (a docutils id cannot start with
  a digit) — the section now carries an explicit `(ring-log)=` target; the
  `docs/…` links of the included `CONTRIBUTING.md` resolve inside the site
  (`:relative-docs:`). `docs/data_format.md` documents `metrics.json` and the
  real layout of `sysid_best_params.json` (the vector is `best_params_raw`, not
  `best_params`; `history`, `bounds`, `args`, `source` were missing), and no
  longer claims `--print-snippet` prints `ArticulationCfg` overrides — it prints
  a Python dict. `docs/troubleshooting.md` gains the PC/NUC version-skew errors
  of `log_start`, a failed ring-log fetch, stale script copies in IsaacLab, a
  missing `pandas` / `matplotlib`, an under-fit run and GPU memory at
  `--num_envs 512`; `docs/installation.md` says to re-run the installer after
  every pull.
- Docs: steps 2-4 of `docs/sysid.md` read the runs from where step 1 now
  writes them (`DATA=~/frankatwin/data/sysid`, set once and reused) instead of
  a placeholder `/data/sysid/`; it also says where the fit, the replay and the
  comparison put their output.
- The Reference docs are cut from ten pages to eight. `interfaces.md` and
  `daemon.md` are gone: the daemon's wire protocol, the shm layout, the
  IsaacLab task ids and how to run the daemon now live in `architecture.md`,
  which dropped its process and timing tables; the client method table,
  script table and C++ binary table are gone in favour of `api.md`, `--help`
  and Usage. `troubleshooting.md` keeps every failure as symptom → fix with
  the code pointer, without the cause analysis, and its Build section keeps
  the four failures everyone meets. `data_format.md` is the schemas.
- SysID details states one difference from OmniReset — the validation
  waveform and gains — instead of two, and no longer carries the model-based
  sizing table for `heldout`, which the run on the arm has replaced.

### Fixed
- Documentation claims that no longer matched the code or the results, each
  re-checked against the source it describes. The headline sim-to-real number
  in `README.md` and the docs landing page was from the superseded
  v3-multiband fit; it now quotes the fit this version documents (low-band
  chirp, validated on `multiband_heldout`: joint-position MSE
  3.0 × 10⁻³ rad²). The landing page's line count was off by 2–3.5×.
  `interfaces.md` and `installation.md` still promised five scripts under
  `isaaclab_sysid/scripts/tools/`, which has held four since `traj_weights.py`
  was removed, and `interfaces.md` had the shm segment 8 bytes too large.
  `daemon.md`'s banner showed a two-value `err_delta` that the controller has
  not had for some time. `troubleshooting.md`'s operational rules told the
  reader to stop the `read_*` helpers before starting the daemon, contradicting
  the rest of the page, `architecture.md` and `src/fci_lock.h`, which only
  `osc_shm` and `move_to` use.
- `set_gains`, `set_ee_target` and `enable` / `disable` failed under numpy 2.x
  with `only 0-dimensional arrays can be converted to Python scalars`. All
  three read the command block back to re-publish the fields they do not
  change, but indexed it as `prev["kp_pos"]` instead of `prev["kp_pos"][0]`:
  `read_command()` returns a shape-(1,) structured array, and numpy 2.0 turned
  `float()` on a shape-(1,) array from a DeprecationWarning into a TypeError.
  `snapshot_gains` / `restore_gains` already indexed correctly, which is why
  the existing tests passed. `tests/test_command_roundtrip.py` now covers the
  three methods.
- `doctor` reports numpy's version on the `python` line. Nothing requires a
  particular major version, but behaviour differs across them, so it belongs in
  the report people already paste when something is off.
- `sysid_franka_osc.py` advanced one 1 ms tick per CSV row, so a 50 Hz run was
  replayed 20× too fast and the loss compared sim time `k` ms against real time
  `20k` ms. The fit now uses the same zero-order-hold time base as
  `replay_python_csv_sim.py` (50-tick warmup, `round(Δt / 1 ms)` ticks per row,
  sample then re-target); a fit rollout matches the replay of the same
  parameters to < 0.003° per joint. Parameters fitted with the affected version
  should be re-fitted.

### Removed
- The 0.30 m/s Cartesian speed and 0.50 rad/s angular rate conventions, and the
  four pre-flight checks that warned against them (`cart_impedance.py`,
  `gen_excitation_traj.py`, `gen_chirp_traj.py`,
  `frankatwin.excitation.chirp`). The pair was inherited from the UR5e
  `collect_sysid_data.py` pipeline this excitation code was ported from, not
  from Franka: libfranka's own ceilings are 3.0 m/s and 2.5 rad/s
  (`franka/rate_limiting.h`), roughly 10x and 5x higher. They were also
  mis-calibrated against the configurations this repo ships and documents --
  `multiband` peaks at 0.3012 m/s and `chirp` at 0.457 m/s / 1.47 rad/s, so
  both known-good excitations warned about themselves on every run.

  The peak-rate diagnostics stay, now printed without a threshold: they
  describe the commanded trajectory (`dx_des` is its analytic derivative),
  which is worth seeing before committing an excitation to the robot.
  `osc_shm`'s per-tick tracking-error clamp is the safety net that actually
  runs.
- The `v3` multiband profile, `--high-band-ratio` and the `high_band_ratio`
  sidecar field. `v3` was the excitation of the earlier, superseded protocol
  (fit on a two-tone multiband, validated on a chirp); the documented workflow
  fits on the low-band chirp and validates on `heldout`, and no collected run
  in `data/sysid/` uses `v3`. `heldout` is now the only multiband profile and
  the default of `--profile`, so `python examples/cart_impedance.py --mode
  multiband` without `--profile` produces the held-out run instead of the old
  two-tone design. `build_multiband_trajectory`'s defaults follow, and its
  `high_band_ratio` parameter is gone (every caller passed keywords).
  `POS_FREQS` / `ORI_FREQS` are gone from `frankatwin.excitation`; the tone
  table is `HELDOUT_FREQS`.
- The `Multiband (v3)` figures and the v3 column of the excitation table in
  SysID details, and the "SysID v3 / v4" generation names in code comments
  and docs. The sidecar `controller` strings `python_v4_chirp_excitation` and
  `v4_chirp_excitation_target` are unchanged: collected runs carry them.

### Changed
- Sysid workflow (`docs/sysid.md`, `docs/sysid_details.md`, `docs/usage.md`):
  fit on the two chirp bands (kp 200/20), validate on the `heldout` multiband
  (kp 500/30) — three runs, all recorded with `--log-1khz`. The stiffer-gain
  low-band held-out run is dropped; the published results come from the
  earlier protocol (fit on the v3 multiband, held-out chirp) and are labelled
  as such. `docs/index.md` and the README no longer claim that the IsaacLab
  controller shares the real one's torque slew limit and torque limits: it has
  no slew limit and clamps at 100 N·m, against 800 N·m/s and 87 / 12 N·m in
  `osc_shm`.
- **Breaking.** `build_multiband_trajectory` returns six arrays, not five: the
  (N, 2) tilt `(rx, ry)` follows `yaw` and `roll` (all zero for v3).
- `cart_impedance.py` and `gen_excitation_traj.py` report the multiband
  reference's largest orientation offset as measured on the quaternions
  (0.474 rad for v3 at a tool-down pose) instead of the bound
  `(amp_yaw + amp_roll) · (1 + high_band_ratio)` = 0.54 rad, which `sine` mode
  printed too although it holds the orientation.
- **Breaking.** `examples/gripper.py --width` is metres, not a fraction of the
  stroke. `--width 0.42` used to mean 42 % (33.6 mm) and now means 42 cm, which
  is out of range and rejected with a message saying so. `--width-m` stays as a
  hidden alias. Nothing else in the project expressed a length as a fraction.
- `gripper.grasp_speed` 0.5 -> 0.1 m/s. The Franka Hand product manual (1.2,
  Technical Data) gives "Travel Speed (per finger) 50 mm/s"; both fingers move,
  so the width closes at up to 0.1 m/s, which is also what libfranka's own
  examples pass. 0.5 was 5x over and the firmware was silently capping it, so
  the documented "closes at up to 0.5 m/s" was never true. `move_speed` was
  already at the ceiling.
- `gripper.grasp_force` is documented as **30-70 N**, not `(0, 70]`: the manual
  states the continuous force is "adjustable 30-70 N", so asking for less than
  30 does not buy a gentler hold. Use `--close-width` to stop the jaws early
  instead. The value itself is unchanged and `gripper_cmd` still only rejects
  <= 0.

### Changed
- `robot.yaml` ships `control.error_delta_pos: 0` (pure impedance) instead of
  `0.05`, so the file finally agrees with `osc_shm`, which has always compiled
  in `0.0` for the same stated reason ("pure impedance to match the unclipped
  sim"). Until the daemon began restoring gains after every controller start
  (c5bac75), `osc_shm`'s 0 is what actually ran, and it is what the v3 sysid
  data was collected under; the restore silently flipped the clamp on.

  The clamp does two things at once: it bounds the controller's push to
  `kp_pos * error_delta_pos` and aborts the loop when the unclipped error
  exceeds it. At `kp_pos` 200 a 0.05 clamp allows 10 N, which cannot follow
  `multiband`'s 0.30 m/s reference, so `cart_impedance.py --mode multiband`
  aborted after ~70 ms and the watchdog relaunched into the same abort -- the
  arm visibly starting and stopping. The `config.py` fallback moves to 0 as
  well. Set a positive value to get the clamp and the abort back.

### Changed
- **Breaking.** The orientation channel is pure impedance: `error_delta_rot` is
  gone everywhere — the per-tick `|e_ori|` clip and the orientation
  tracking-error abort in `osc_shm`, the `ShmCommand` field (the struct is now
  **112 B**, `enabled` at offset 104), the `set_gains(error_delta_rot=)`
  argument on `LocalController` / `FrankaTwinClient` and its daemon wire field,
  `robot.yaml`'s `control.error_delta_rot`, `--err-delta-rot` on
  `examples/cart_impedance.py` and `examples/policy_loop.py`, and the
  `ORI_TRACK_ABORT_RAD` pre-flight warnings in the excitation scripts. Position
  keeps its clamp and abort (`error_delta_pos`). Rebuild and restart the daemon:
  an old binary and a new one disagree on the command-block layout. The
  remaining orientation safety net is the per-joint torque clamp, the slew
  limiter, and libfranka's collision reflex and hard limits — with no cap on
  `Kp_ori · e_ori`, consider lowering `collision.torque_threshold` (100 N·m was
  chosen on the assumption that the controller's own push was bounded).
- **Breaking.** One pacing knob for both `move_to` modes: `--q-max-speed`, a
  per-joint velocity cap in rad/s, range (0, 1.25], default 0.5. It replaces
  `--speed-factor` and `--duration` on the binary, `speed_factor=` / `duration=`
  on `LocalController` / `FrankaTwinClient` (`q_max_speed=`), the `speed_factor`
  / `duration` wire fields (`q_max_speed`), `--speed` / `--duration` on
  `examples/move_to.py` (`--q-max-speed`), and `reset.joint_speed_factor` /
  `reset.pose_duration` in `robot.yaml` (`reset.q_max_speed`). The default
  reproduces the old behaviour exactly: 0.5 rad/s is `speed_factor` 0.2 against
  MotionGenerator's largest `dq_max_` (2.5), and the 1.25 ceiling is the old
  `speed_factor <= 0.5`. A `robot.yaml` still carrying the old keys falls back
  to the default rather than erroring.

  Motion time is now derived from the travel in both modes, so a longer move
  takes longer instead of moving faster -- which a fixed duration could not do.
  For `--pose` the cap is **approximate**: libfranka owns the IK, so `move_to`
  estimates the joint displacement from the Jacobian at the start pose
  (damped least squares) and sizes the min-jerk profile from that. The estimate
  degrades over large reorientations and near singularities; the binary says so
  on stdout, and the docs repeat it.
- Installation is conda-only: one `frankatwin` env per machine, libfranka from
  conda-forge matched to the robot's FCI protocol (system 5.9 → `libfranka=0.20`;
  a mismatch only shows up when a session is opened). `CMakeLists.txt` finds
  conda's boost of any version and only when Pinocchio is linked.
- `sysid_franka_osc.py` replays the trajectories in parallel env blocks:
  `--num_envs` is the CMA population, and the sim holds `num_envs × trajectories`
  envs. Each block starts from its run's `q_init` and uses its run's gains from
  the sidecar, so runs recorded under different gains (chirp at kp 500 / 30,
  multiband at 200 / 20) can be fitted together; the shared-gains check is gone.
- The optimizer steps through the new `FrankaTwinSysidEnv.physics_step()`
  (controller + physics only, without `DirectRLEnv.step()`'s dones / rewards /
  observations and their per-tick GPU→CPU sync) and scores only at the CSV
  sample instants. The env no longer fetches the mass matrix in
  `task_impedance` mode without nullspace, where the controller does not use it.

## [0.2.0] — 2026-09-03

First public release, renamed from the internal `panda_control` repository.

### Changed
- Package renamed `panda_control` → `frankatwin`; `RemotePandaClient` →
  `FrankaTwinClient`, `LocalPandaController` → `LocalController`,
  env var `PANDA_CONFIG` → `FRANKATWIN_CONFIG`, shm segment `/panda_osc` →
  `/frankatwin_osc`, IsaacLab tasks `Isaac-UW-Franka-*` → `Isaac-FrankaTwin-*`.
- `cart_impedance.py --mode step5d` → `--mode multiband`;
  `build_step5d_trajectory` → `build_multiband_trajectory`.
- `gen_excitation_traj.py` / `gen_chirp_traj.py`: `--base-sidecar` is required;
  outputs default to `data/`.
- `apply_sysid_params.py`: default `--replay-script` is the shipped
  `replay_python_csv_sim.py`.

### Fixed
- The daemon now restores gains, error clamps and `enabled` after every
  `osc_shm` start. Previously each restart (inside `move_to_q`/`move_to_pose`
  and, since the watchdog landed, any automatic relaunch) silently reset the
  controller to `osc_shm`'s built-ins (kp 200 / 20, clamps off). Initial
  values come from `robot.yaml → control:`.

### Removed
- Experiment scripts that depended on an unpublished IsaacLab task
  (`six_dof_pose_test`, `fixed_delta_pose_test`, `two_phase_smoke_test` and
  their compare tools). They remain on the `v0.1` branch.

### Added
- `python -m frankatwin.doctor`: environment / connectivity check with a
  hint per failing line (RT kernel, rtprio, binaries + ldd, FCI port, other
  FCI clients, daemon ping, state stream).
- `examples/policy_loop.py`: fixed-rate policy skeleton on task impedance;
  `frankatwin.quat`: wxyz quaternion helpers (product, rotvec, osc_shm-style
  orientation error).
- `examples/move_to.py`: one position-control script — home (default),
  `--target-joints`, or `--target-ee x y z qw qx qy qz` (replaces
  `reset_home.py` / `move_to_q.py`).
- `frankatwin.excitation` package: the reference math moved out of
  `scripts/` (no more `sys.path` hacks); `scripts/gen_*_traj.py` are now
  just the command line around it.
- Tests for config resolution, excitation builders and the CLI.
- Apache-2.0 license, third-party notices, citation metadata, CI, and the
  `docs/` set (installation, architecture, usage, sysid, data format,
  troubleshooting).

## [0.1.x] — 2026-05 … 2026-07 (internal)

- 1 kHz Jacobian-transpose Cartesian impedance controller (`osc_shm`) with
  POSIX-shm command/state interface, PC↔NUC ZMQ daemon, joint/pose `move_to`.
- Torque slew-rate limiter (fixes `controller_torque_discontinuity` reflex at
  setpoint jumps), controlled stop, configurable collision thresholds,
  payload `setLoad`, shm v3 with measured `tau_J`, osc_shm watchdog,
  Pinocchio linking for libfranka ≥ 0.14.
- CMA-ES system identification (armature, static/dynamic/viscous friction,
  motor delay) with v3 multi-band and v4 chirp excitation; self-contained
  IsaacLab extension.
