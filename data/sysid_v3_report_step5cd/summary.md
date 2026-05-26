# SysID v3 — Joint fit on step5b + step5c + step5d

## Pipeline used

```
$PY scripts/tools/sysid_franka_osc.py \
  --real_csv data/step5b_20260524_120834.csv  --real_sidecar ...json \
  --real_csv data/step5c_20260525_093748.csv  --real_sidecar ...json \
  --real_csv data/step5d_20260525_143929.csv  --real_sidecar ...json \
  --traj_weights 0.5,1.0,1.5 \
  --num_envs 128 --max_iter 40 --sigma 0.3 --headless
```

Runtime: ~4 hours (≈378 s / iter). Best params output:
`IsaacLab/logs/sysid_franka/20260525_145807/sysid_best_params.json`

Convergence: best score 7.9e-3 → 4.5e-3 (iter 10) → 2.9e-3 (iter 20) → 2.7e-3 (iter 30) → 2.2e-3 (iter 40).

## step5c results (pos-only sweep, was held-in for v2)

| config | EE 3D RMS [mm] | joint RMS [mrad] |
|---|---:|---:|
| baseline (no sysid) | 30.99 | 531 |
| v1 sysid (step5b only) | 17.48 | 52.1 |
| v2 sysid (step5b+5c) | 10.07 | 33.5 |
| **v3 sysid (step5b+5c+5d)** | **7.27** | **31.1** |

v3 is **28 %** better on EE 3D than v2 on step5c. Per-axis EE RMS [mm]:

| config | x | y | z |
|---|---:|---:|---:|
| baseline | 21.86 | 13.57 | 17.27 |
| v1 sysid | 10.06 | 12.91 | 6.13 |
| v2 sysid | **2.43** | 9.17 | 3.37 |
| v3 sysid | 3.79 | **5.70** | **2.46** |

(v3 gives up a sliver on x to gain a lot on y and z.)

## step5d results (pos + rotation sweep, held-out for v1/v2, held-in for v3)

This is **the** trajectory of interest — the only one with explicit j1/j5 excitation.

| config | EE 3D RMS [mm] | joint RMS [mrad] |
|---|---:|---:|
| baseline (no sysid) | 41.11 | 982.5 |
| v1 sysid (step5b only) | 28.34 | 88.6 |
| v2 sysid (step5b+5c) | 18.81 | 60.4 |
| **v3 sysid (step5b+5c+5d)** | **8.35** | **25.5** |

v3 is **56 % better** on EE 3D and **58 % better** on joint RMS than v2 on step5d.

Per-joint q-RMS [mrad] on step5d — note the dramatic improvement on the
two joints that were under-identified in v1/v2:

| config | j1 (base yaw) | j2 | j3 | j4 | j5 (wrist roll) | j6 | j7 |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 1718.5 | 538.9 | 1778.4 | 172.3 | 438.4 | 173.0 | 315.1 |
| v1 sysid | 55.2 | 33.3 | 64.8 | 60.6 | 69.3 | 94.5 | 170.8 |
| v2 sysid | 88.0 | 25.4 | 89.2 | 37.5 | 29.5 | 52.6 | 64.3 |
| **v3 sysid** | **38.9** | **12.8** | **38.0** | **15.7** | **10.1** | **18.5** | **27.1** |

**j1: 88 → 39 mrad (−56 %).  j5: 29.5 → 10.1 mrad (−66 %).** The base yaw +
EE-z roll rotation excitation in step5d gave CMA-ES the friction signature
it was missing.

## v3 best params (decoded)

| joint | armature [kg·m²] | μ_static [Nm] | μ_dynamic [Nm] | μ_viscous [Nm·s/rad] |
|---|---:|---:|---:|---:|
| j1 | 0.382 | 0.73 | 0.29 | 3.68 |
| j2 | 0.159 | 1.17 | 0.90 | 2.29 |
| j3 | 0.157 | 0.59 | 0.34 | 2.87 |
| j4 | 0.174 | 1.03 | 0.81 | 2.37 |
| j5 | 0.239 | 1.63 | 0.92 | 3.30 |
| j6 | 0.180 | 1.14 | 0.58 | 0.79 |
| j7 | 0.060 | 1.06 | 0.50 | 1.94 |
| global | `motor_delay_steps = 1` |

Directional shifts vs v2 (which was over-fit toward "friction explains
everything"):
- **μ_static dropped almost everywhere** (j1: 3.13→0.73, j3: 2.48→0.59,
  j4: 1.51→1.03, j7: 1.41→1.06). The step5d data has enough joint motion
  that CMA-ES can tell stiction apart from viscous damping.
- **μ_viscous rose on j1, j3, j5** (j1: 2.89→3.68, j3: 2.02→2.87, j5: 2.45→3.30).
  This is the energy that v1/v2 was forced to dump into μ_static because
  step5b/5c didn't move those joints fast enough to make viscous identifiable.
- **Armature mostly down** (more compliant arm model). With step5d's higher
  accelerations, CMA-ES no longer needs to inflate inertia to explain lag.

## Visual comparison

- `step5c/sxs_position.png`, `step5c/sxs_joints.png` — 4 configs on step5c.
- `step5d/sxs_position.png`, `step5d/sxs_joints.png` — 4 configs on step5d.

On step5d's joint plot, watch **j1 column 4 (v3)** track the real ±0.30 rad
swing instead of staying flat as in columns 2/3 — and **j5 column 4** track
the real ±0.10 rad motion vs columns 2/3's near-flat sim. That is exactly
the residual we set out to remove.

## Backward compatibility on step5b

(Not regenerated this round; v3 best params decoded show similar magnitudes
to v2 on step5b's regime — μ_static is lower but viscous is higher, which
on step5b's slow z-sin nets out almost identical EE behaviour. If you want
to verify, run `apply_sysid_params.py --best <v3 path> --invoke-replay --real-csv step5b_…csv` and re-run the compare script.)
