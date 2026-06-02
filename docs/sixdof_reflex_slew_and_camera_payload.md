# 6-DOF 测试:相边界 Reflex 根治、sim2real 对齐、相机负载补偿

> 日期:2026-05-31  ·  范围:`panda_control`(真机)+ `IsaacLab` Franka-Debug-v2(仿真)
> 起因:`two_phase_smoke_test` / `six_dof_pose_test` 在相边界触发 libfranka Reflex,
> 机械臂锁死、`move_to_q` 回 home 被拒。

---

## 1. 结论速览(TL;DR)

| 问题 | 根因 | 修复 | 验证 |
|---|---|---|---|
| 相边界 Reflex,osc_shm 死、回 home 被拒 | `controller_torque_discontinuity`:Jᵀ 阻抗律在 setpoint 跳变时吐出阶跃力矩,超 libfranka 的 1000 N·m/s 力矩速率上限 | osc_shm 加**力矩 slew 限速**(默认 800 N·m/s) | `--settle 0` 也能跑完 7 相、回 home 成功、相内轨迹零扰动 |
| "settle 能修 Reflex" | **错误假设**(从未被数据验证) | 删除;settle 默认关 | 带 settle 的历史 run 在同一处冻死 |
| sim2real 位置残差 | 真·动力学差(sysid 不泛化 + z) | 对比工具 + 50Hz 重锚消除采样率混淆 | 朝向 RMS 1.5° 对齐;位置残差非设置假象 |
| 装相机后 z 下垂 ~100mm | 相机负载未进 libfranka 重力补偿 | osc_shm `setLoad`(config 驱动) | 标定有效负载 **0.15kg**,z 偏差 −27mm → **−1.3mm** |

---

## 2. Reflex 根因与力矩 slew 修复

### 2.1 诊断

- 失败现象:相边界后 osc_shm 死,SUB 状态流冻结,`move_to_q` 报
  `command not possible in the current mode ("Reflex")`。
- 用 daemon `-v` 拿到真因:
  `franka::Exception: ... motion aborted by reflex! ["controller_torque_discontinuity"]`,
  且 `control_command_success_rate: 1`(排除丢包)。
- 逐 tick CSV(q/dq/seq)证据:死亡帧关节离限位余量最小 0.73 rad、`dqmax≈0.004 rad/s`(近静止)
  → **排除关节限位、速度反转**;纯粹是**指令力矩的阶跃**。
- 定量:外伸构型下一个 +y 末端力(7.5N)几乎全压在底座关节 1(力臂 ~0.38m)→ 单个 1kHz tick 内
  Δτ₁ ≈ 2.9 N·m → dτ/dt ≈ 2900 N·m/s ≫ 1000 → reflex。

### 2.2 为什么 settle 治不了

带 settle 与不带 settle 的历史 run **都精确卡在 `phase2_y` 启动那一拍**:settle 把速度干净
降到 0,但 reflex 触发于"引入 +y 命令"那一下的**力矩阶跃**,与速度无关 → settle 只是把死亡
时刻从 tick 102 推迟到 116。`automaticErrorRecovery()` 同理只是治标(清锁存,不消除原因)。

### 2.3 修复(控制器层,根治)

- **真机** [`src/osc_shm.cpp`](../src/osc_shm.cpp):在 `tau_cmd` 幅值钳位之后、发布之前,逐关节限速
  `|Δτ| ≤ max_torque_rate · dt`。`tau_prev` 跨 tick 持久化、初值 0。CLI `--max-torque-rate`
  (默认 800 = 留 20% 余量;`≤0` 关闭)。相内 dτ/dt 远低于阈值 → 限速器休眠,**不改变被对比的轨迹**。
- 附带修复:`<signal.h>` → `<csignal>`(latent `std::signal` 编译错,重编时才暴露)。
- **仿真** Franka-Debug-v2:镜像同一限速(见 §4)。

验证:`--settle 0`(原失败复现条件)跑通两相,phase2 真位移 15cm,回 home 成功,NUC 无 reflex;
相内 phase1 修复前后逐拍 max |Δx|=2.6mm(读缓存抖动量级)→ **零扰动**。

---

## 3. 脚本硬化(both six_dof & two_phase_smoke)

[`examples/six_dof_pose_test.py`](../examples/six_dof_pose_test.py) /
[`examples/two_phase_smoke_test.py`](../examples/two_phase_smoke_test.py):

- **settle 默认 OFF**(`--settle` 可选,仅供实验),对齐仿真,7 相 / 14s / 700 拍。
- **SUB seq 停滞看门狗**:osc_shm 一死立刻报"死在第几 tick / 哪个相"并中止 + 标记 INVALID,
  不再静默采集冻结数据(就是这条抓出了"装相机时 osc_shm 死掉"和"User Stop 未松开")。
- **q/dq/seq 写入 CSV**;**写日志移到回 home 之前** + 回 home 改为非致命(失败也保住轨迹)。
- meta sidecar 记录 `tag` / `settle_s` / `froze`。

> ⚠️ 2026-05-31 之前的 `data/real_sixdof_kp500_*` CSV 在 phase1 之后全部冻结(vmax≈0),
> 对 sim2real 无效,已重采。

---

## 4. sim2real 对齐(matched-gain 7 相对比)

仿真:IsaacLab `Franka-Debug-v2` /
`scripts/environments/franka_debug_v2_six_dof_test.py`(repo `/home/tao/Projects/IsaacLab`)。

**全部对齐项**:kp_pos=500 / kp_ori=30;kd=2·√kp(真机 osc_shm 默认 == 仿真
`get_deriv_gains`,`task_deriv_scale=1.0`);7 相无 settle 14s;**力矩 slew 800 N·m/s**
(仿真 `FrankaDebugV2EnvCfg.torque_rate_limit_nm_per_s`,base/v0/v1 默认 0=关);
1kHz 控制;**chase 重锚 50Hz**(`--reanchor-hz`,镜像真机 PC 50Hz set_ee_target)。

仿真侧改动:
- `Franka_Debug/franka_debug_env_cfg.py`:新增 `torque_rate_limit_nm_per_s`(默认 0.0)。
- `Franka_Debug/franka_debug_v2_env_cfg.py`:v2 覆盖为 800.0。
- `Franka_Debug/franka_debug_env.py`:`_apply_action` 力矩 slew + `_reset_idx` 清零 prev。
- `scripts/.../franka_debug_v2_six_dof_test.py`:去掉硬编码 settle;`--reanchor-hz` 默认 50,
  重锚之间保持 target(镜像真机)。

**结果**:
- **朝向 RMS ~1.5°** → 控制链(增益/kd/slew/相位)确认对齐。
- 位置平移相 RMS ~32mm,且把重锚从 1kHz 改到 50Hz **不缩小反略增**(27→32mm)→ **采样率被排除**,
  残差是**真·动力学差**:真机 x/y 多走 ~30%,z 方向相反 → 指向各向异性 sysid 参数未泛化到 6-DOF chase。
- 对比脚本:[`scripts/compare_sixdof_sim_real.py`](../scripts/compare_sixdof_sim_real.py);
  图在 `data/compare_50hz/`(50Hz 干净版)。

> 注:旋转相位置带 real-vs-sim EE 参考点混淆(libfranka kEndEffector vs sim
> panda_fingertip_centered);平移相 + 朝向不受影响。

---

## 5. 相机负载补偿(ZED Mini)

### 5.1 setLoad 链路(config 驱动)

libfranka 三层负载:`m_total = m_arm + m_ee(Desk) + m_load(setLoad)`。`setLoad` 只设
**load 项**(替换,不累加),**不影响 Desk 的 Franka Hand 配置**。

- [`src/osc_shm.cpp`](../src/osc_shm.cpp):`robot.setLoad(mass, com, inertia)`(control 之前),
  CLI `--load-mass/--load-com/--load-inertia`;**非致命 try/catch**(负载非法只 WARN 不崩 daemon)。
- [`python/panda_control/config.py`](../python/panda_control/config.py):新增 `LoadConfig`。
- [`python/panda_control/local_controller.py`](../python/panda_control/local_controller.py):
  daemon 启动 osc_shm 时透传负载参数。
- [`config/robot.yaml`](../config/robot.yaml):`load:` 块(默认 mass=0 关闭,向后兼容)。
- [`src/read_load.cpp`](../src/read_load.cpp):一次性诊断,打印 `m_ee/m_load/m_total`
  (停 daemon 后 `./build/read_load <ip>`,用于确认 Desk 是否已补偿 Hand)。

### 5.2 标定有效负载 = 0.15 kg

用 z 方向的**对称过补/欠补签名**夹出真值(不依赖 com,因为重力**力**只取决于 mass):

| 补偿质量 | 平移相 z 相对 nocam | 含义 |
|---|---|---|
| 0 kg(未补偿) | **−27.1 mm** | 欠补,下垂 |
| 0.3 kg | **+27.3 mm** | 过补,上顶 |
| **0.15 kg** | **−1.3 mm** ✅ | 命中 |

逐相单独反解一致落在 0.147–0.158 kg(均值 0.153);静态 home z 偏差也从 −5.9mm(未补偿)/
+0.6mm(0.3kg)收敛到 **−0.1mm**(0.15kg)。物理上也合理:ZED Mini 裸机 63g + 支架/接头 ≈ 0.15kg。

最终 config:
```yaml
load:
  mass: 0.15
  com: [0.0, 0.0, 0.05]   # 简化为沿工具轴;横向清零避免注入错误力臂
  inertia: [3.0e-4, 0,0, 0, 3.0e-4, 0, 0,0, 3.0e-4]   # 量级正确;本测试用不到(见下)
```

### 5.3 com / inertia 说明

- **com**:只改重力**力矩(力臂)**→ 只影响**旋转相(phase4-6)**的朝向/关节力矩分配,
  不影响 z 下垂。相机大致在工具轴上时简化为 `[0,0,z]` 最稳;旋转相若有方向性残差再调 x/y。
- **inertia**:重力补偿**不看** inertia;我们的阻抗律无 apparent-mass 投影,inertia 只经
  Coriolis(∝ dq²)进入,本测试 ~0.1 rad/s 下可忽略。当前 `3e-4` 对角量级正确,留着即可。
  仅在将来高速运动 / sim 严格对齐时才需算准。

---

## 6. 复现命令

### 真机(NUC + PC)
```bash
# NUC:重编(吃进 slew + setLoad + read_load + <csignal> 修复),启动 daemon
cd ~/Projects/panda_control && cmake --build build
python -m panda_control.daemon -v
#   banner 应有:[osc_shm] tau_rate = 800 Nm/s (slew limit)
#               [osc_shm] load     = 0.15 kg, com=[0,0,0.05] m (gravity-compensated)

# PC:采数据(确认打印 'SUB cache alive' = osc_shm 活着)
python examples/reset_home.py
python examples/six_dof_pose_test.py --kp-pos 500 --kp-ori 30 --tag <withcam|nocam> \
    --log data/real_sixdof_<tag>_$(date +%Y%m%d_%H%M%S).csv
```

### 仿真(IsaacLab)
```bash
conda activate isaaclab          # base/panda 环境没有 isaaclab
cd ~/Projects/IsaacLab
./isaaclab.sh -p scripts/environments/franka_debug_v2_six_dof_test.py \
    --task Franka-Debug-v2 --kp_pos 500 --kp_rot 30 --reanchor-hz 50 \
    --num_envs 1 --headless --tag v2_50hz_kp500
# 输出:logs/franka_debug_v2/videos/trajectory_<tag>.csv
```

### 对比(需 pandas+matplotlib → 用 isaac/isaaclab 环境 python)
```bash
PY=/home/tao/miniconda3/envs/isaac/bin/python
# sim vs real
$PY scripts/compare_sixdof_sim_real.py --real-csv <real.csv> --sim-csv <sim.csv>
# real vs real(相机 vs 无相机 / 不同负载)
$PY scripts/compare_real_runs.py \
    --csv-a <baseline.csv> --label-a nocam \
    --csv-b <other.csv>    --label-b withcam
```

---

## 7. 关键数据产物

| 路径 | 内容 |
|---|---|
| `data/real_sixdof_20260531_114913.csv` | nocam 基线(7 相,有效) |
| `data/real_sixdof_withcam_20260531_123540.csv` | 有相机,**未补偿**(z 下垂 ~100mm) |
| `data/real_sixdof_withcam_loadset_20260531_134642.csv` | 有相机,补偿 **0.3kg**(过补 +27mm) |
| `data/real_sixdof_withcam_load015_20260531_135947.csv` | 有相机,补偿 **0.15kg**(命中 −1.3mm) |
| `data/compare_cam015_vs_nocam/` | 最终相机标定对比图 |
| `data/compare_50hz/` | sim2real 干净对比图(50Hz 重锚) |

---

## 8. 待办 / 后续

- **sysid 不泛化**:当前 sysid 在 step5b/c/d 上拟合,6-DOF chase 位置残差 ~32mm。
  考虑用这条干净的 chase 轨迹重新拟合 / 检查(x/y 偏迟钝、z 行为不符)。
- **com 微调**:若相机标定后旋转相(phase4-6)仍有方向性残差,按力臂反推微调 `com`。
- **sim 加相机**:若要 sim 也复现带相机动力学,在 Franka-Debug-v2 里加同样的末端负载。
- **操作规范**:装/拆末端负载前先停 daemon(别在 osc_shm 控制中硬上负载);
  采数据前确认 User Stop 已松开、脚本打印 `SUB cache alive`。
