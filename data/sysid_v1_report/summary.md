# SysID Validation v1 Report

## Held-out summary

| metric | real | sim baseline | sim sysid |
|---|---:|---:|---:|
| err_pos_rms_xyz [mm] | `[3.347, 1.062, 15.667]` | `[1.063, 0.128, 5.265]` | `[1.810, 0.139, 15.801]` |
| err_pos_max_z [mm] | 24.508 | 9.310 | 24.722 |
| theta_max [mrad] | 31.770 | 3.186 | 32.165 |

- Held-out success criterion (`|z_sysid-z_real|/z_real < 30%`): **PASS**

## Ablation contribution (z RMS)

| config | err_pos_rms_z [mm] | delta_to_all_on [mm] |
|---|---:|---:|
| all_on | 15.801 | +0.000 |
| armature_off | 15.780 | -0.021 |
| delay_off | 15.801 | -0.000 |
| friction_off | 9.480 | -6.321 |
| viscous_off | 15.244 | -0.557 |

## V2 recommendations

- ablation影响<1%的组: armature_off, delay_off，v2可考虑锁死该组参数减少搜索维度。
- held-out泛化达标（z-rms相对误差<30%），可进入多轨迹联合拟合验证稳定性。

## Artifacts

- ablation inputs: `/home/tao/Projects/panda_control/data/step5c_proxy_ablation/ablation_outputs.json`
- bar chart: `/home/tao/Projects/panda_control/data/sysid_v1_report/ablation_rms_z.png`
