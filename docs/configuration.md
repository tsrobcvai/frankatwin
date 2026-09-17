# Configuration

One file, `config/robot.yaml`, read by every process on every machine. Pass
another with `--config` on any script, or set `FRANKATWIN_CONFIG=/path/to.yaml`;
relative `paths.build_dir` resolves against the repo root. The daemon reads it
once at start — restart it after edits.

| key | default | meaning |
|---|---|---|
| `network.nuc_host` | `172.16.0.1` | Daemon address as seen from the PC. |
| `network.cmd_port` / `state_port` | 5555 / 5556 | REQ/REP and PUB ports. |
| `network.state_cache` | 256 | Client-side frame cache depth. |
| `robot.ip` | `172.16.0.2` | FCI address (NUC side). |
| `robot.init_q` | Franka home | `examples/move_to.py` default (home) target, 7 floats [rad]. |
| `control.kp_pos` / `kp_ori` | 200 N/m / 20 N·m/rad | Initial gains the daemon writes at startup; runtime override via `set_gains` (persists across restarts). Also the defaults of `cart_impedance.py --kp-*`. |
| `control.kd_pos` / `kd_ori` | `null` | `null` → `2√kp` (`osc_shm`'s auto rule). |
| `control.error_delta_pos` | 0.05 m | Initial per-tick position error clamp + abort. `0` → pure impedance (as in sim). The orientation channel is always pure impedance: no clamp, no abort. Runtime override via `set_gains(error_delta_pos=)` or `python examples/cart_impedance.py --err-delta-pos`. |
| `collision.torque_threshold` / `cartesian_threshold` | 100 N·m / 100 N | `setCollisionBehavior` thresholds (all entries). |
| `paths.build_dir` | `build` | Where `osc_shm` / `move_to` live (relative to repo root). |
| `paths.shm_name` | `/frankatwin_osc` | POSIX shm name. |
| `reset.q_max_speed` | 0.5 rad/s | Per-joint velocity cap for **both** `move_to` modes, (0, 1.25]. Motion time follows from the travel. Approximate for `--pose` — see [Usage](usage.md). |
| `load.mass` / `com` / `inertia` | 0 / 0 / 0 | Extra payload for `setLoad`; see the comments in the file. |
| `gripper.enabled` | true | `false` makes every `gripper_*` command fail fast (no Franka Hand). |
| `gripper.move_speed` / `grasp_speed` | 0.1 / 0.5 m/s | Finger speed for `gripper_open` (`Gripper::move`) / `gripper_close` (`Gripper::grasp`). |
| `gripper.grasp_force` | 70 N | Squeeze force once the fingers stall — the knob for how hard an object is held. (0, 70]; 70 is the Franka Hand's rated continuous maximum, lower it for anything crushable. |
| `gripper.grasp_width` | −0.01 m | Width the fingers drive towards when closing. Past full closure ⇒ they always reach the object and the object sets the resting width. |
| `gripper.epsilon_inner` / `epsilon_outer` | 0.08 / 0.08 m | Band around `grasp_width` inside which libfranka reports `is_grasped` / `result: true`. 0.08 = whole stroke = every stall counts (deoxys behaviour); tighten to make "grasped" meaningful. |
| `gripper.max_width` | 0.08 m | Full stroke; `gripper_open()` default and the reference for `examples/gripper.py --width FRAC`. |

`FRANKATWIN_CONFIG=/path/to.yaml` overrides the default location.
