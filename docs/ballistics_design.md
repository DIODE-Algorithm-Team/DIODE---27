# 弹道参数与求解方案（Q4 详细答复）

> 背景：DIODE-27 当前用**无阻力解析解** [`tools::Trajectory`](../tools/trajectory.cpp)，而 `弹道解算及云台角度求解算法.txt` 要求二次阻力 + RK4 + 三未知量 Newton。
> 本文回答「参数怎么给、怎么标、怎么和云台通信对上」，是工作流 C 的实施依据。
> 规划见 [auto_aim_upgrade_plan.md](auto_aim_upgrade_plan.md)。

---

## 1. 结论先说

| 项 | 结论 |
|---|---|
| 参数化 | **混合方案**：配置写物理四项 `m / d / C_D / ρ`，启动时算出合成系数 `c`；同时允许 `resistance_coefficient` 直接覆盖（实测优先） |
| 缺省值 | `m=3.2 g`、`d=17 mm`、`C_D=0.47`、`ρ=1.204` → `c≈0.0201 m⁻¹`；**但必须实测标定后覆盖** |
| 延迟 | 拆成两个参数：`fire_delay`（发指令→出膛）进拦截方程，`predict_extra_delay`（图像+通信+控制，≈15 ms）进预测时刻 |
| 枪口 | 新增 `muzzle_offset`（相对云台旋转中心），对应 txt 的 `r_GM`；缺省全 0（= 当前行为） |
| 通信 | **串口链路可带前馈，CAN 链路不能**（8 字节只有角度）。MPC 要生效必须走串口或扩帧 |
| 回退 | 求解失败/超时自动退回现有解析解，保证永不输出非法角度 |

---

## 2. 为什么必须引入阻力系数

弹丸受重力与二次阻力：

```
a = [ -c·|v|·vx , -c·|v|·vy , -g - c·|v|·vz ]
c = ρ·C_D·A / (2m) = ρ·C_D·π·d² / (8m)      [单位 1/m]
```

- 该项**没有解析解**，只能数值积分（RK4）；
- `c` 是唯一无法从配置「猜准」的量：它同时包含弹丸质量、迎风面积、阻力系数、空气密度，且实际弹丸带自旋/凹坑，`C_D` 与教科书球体差异很大。

### 2.1 `c` 的取值区间（按常见参数算）

| 质量 | 直径 | C_D | c (1/m) |
|---|---|---|---|
| 3.2 g | 17 mm | 0.30 | 0.0128 |
| 3.2 g | 17 mm | 0.47 | **0.0201** |
| 3.2 g | 16.8 mm | 0.47 | 0.0196 |
| 2.4 g | 17 mm | 0.30 | 0.0171 |
| 2.4 g | 17 mm | 0.47 | 0.0268 |

RMCS 里那个 `kAirResistanceCoefficient = 0.003`（[trajectory_solution.cpp:17](../rmcs_auto_aim_v2/src/module/fire_control/trajectory_solution.cpp#L17)）**明显低于物理估算**，属于经验调小值——这也侧面说明「照抄常数」不可靠。

### 2.2 不同 `c` 的后果（v0=25 m/s，10 m 处，相对无阻力的**附加下坠**）

| c (1/m) | 所需 pitch | 附加下坠 | 飞行时间 | 比真空多 |
|---|---|---|---|---|
| 0（真空，现状） | 4.51° | 0 mm | 0.4013 s | 0 |
| 0.003（rmcs） | 4.60° | 16 mm | 0.4074 s | +6 ms |
| 0.008 | 4.77° | 44 mm | 0.4180 s | +17 ms |
| 0.015 | 5.01° | 86 mm | 0.4333 s | +32 ms |
| **0.0201（物理估算）** | **5.19°** | **118 mm** | **0.4448 s** | **+44 ms** |
| 0.030 | 5.58° | 187 mm | 0.4689 s | +68 ms |

> 小装甲板宽约 135 mm。也就是说：**用错 c，10 m 处可能整块板打飞**；同时飞行时间差 44 ms 会让前哨站（0.8π rad/s）多转 **6.3°**。

### 2.3 初速对下坠的影响（c=0.0201，10 m）

| v0 | 附加下坠 |
|---|---|
| 15 m/s | 373 mm |
| 20 m/s | 190 mm |
| 25 m/s | 118 mm |
| 30 m/s | 82 mm |

低弹速（15–20 m/s）时阻力项是决定性的，**必须**上阻力模型；高弹速（30 m/s）时误差仍有 82 mm，接近半个板。

---

## 3. 参数化方案对比

| 方案 | 配置形式 | 优点 | 缺点 | 结论 |
|---|---|---|---|---|
| A. 单参数 `c` | `resistance_coefficient: 0.008` | 标定最简单、一行 | 换弹丸/换赛季不可解释，无法从物理推 | 备选（作为覆盖项） |
| B. 物理四项 | `m / d / C_D / ρ` | 可解释、可复用、换弹丸只改两数 | `C_D` 仍不准，等于把误差藏在 `C_D` 里 | **必选（作为初值来源）** |
| C. 混合 | 四项算 `c0`，`resistance_coefficient` 可选覆盖 | 既有物理初值又可实测微调 | 多一层合并逻辑（10 行） | ✅ **推荐** |

**取消 `model: vacuum|drag` 开关**：真空模型等价于 `c=0`，用同一个求解器即可，少一条代码路径（`resistance_coefficient: 0` 就退化回现状）。

---

## 4. 配置 schema（全部带缺省，缺键不抛异常）

```yaml
#####-----弹道参数（工作流 C）-----#####
ballistics:
  # 物理四项：用于计算初始 c0（可只给 c 覆盖）
  bullet_mass: 0.0032        # kg
  bullet_diameter: 0.017     # m
  drag_coefficient: 0.47     # 无量纲
  air_density: 1.204         # kg/m^3（20℃，1 atm）
  # 实测覆盖：给数值则忽略上面四项（推荐上线后填实测值）
  resistance_coefficient: 0.0   # 1/m，0 = 用物理四项自动算
  gravity: 9.7833               # m/s^2（沿用现有代码；按场地纬度可调）

  # 时间补偿（两个延迟分开，语义不同）
  fire_delay: 0.030             # s，发开火指令 -> 弹丸离开枪口
  predict_extra_delay: 0.015    # s，图像+通信+控制合计（readme §4.4 实测≈15ms）

  # 枪口相对云台旋转中心（txt 的 r_GM），缺省 0 = 现有行为
  muzzle_offset: [0.0, 0.0, 0.0]   # m，gimbal_link 下

  # 求解器（一般不用改）
  rk4_step: 0.0005           # s
  max_iterations: 8
  max_fly_time: 1.5          # s，超过即判不可解
```

> 兼容性：`configs/*.yaml` 里没有 `ballistics:` 段时，全部使用上述缺省值 → **老配置行为完全不变**（除了 `c` 从 0 变成物理值，这点需在首测时确认）。

---

## 5. 标定三步法（这是 Q4 的核心）

### S1 静态落点法（定 `c`，最重要）

**步骤**：靶纸固定距离 `d∈{3,5,8,10} m`，云台静止、水平零点，记录每发实际落点高度 `y_meas`；与「同角度下的真空预测」比较，得附加下坠 `Δh`。

```
Δh = y_vacuum(d, pitch_used) - y_meas
```

**反解**：对 `c` 做二分（区间 `[0, 0.05]`），使 RK4 解出的落点等于 `y_meas`。

**查表（v0=25 m/s，10 m，可直接对照）**：

| 实测附加下坠 | 反解 c |
|---|---|
| 16 mm | 0.003 |
| 27 mm | 0.005 |
| 44 mm | 0.008 |
| 56 mm | 0.010 |
| 86 mm | 0.015 |
| 118 mm | 0.020 |
| 187 mm | 0.030 |

**验收**：同一 `c` 在 3/5/8/10 m 四处预测误差都 < 20 mm（板宽 135 mm 的 15%）。

### S2 飞行时间法（定 `fire_delay`）

用 240 fps 以上高速相机拍「发弹指令 → 弹丸出膛」，或双帧法（枪口出现弹丸的那一帧）。典型值 **20–40 ms**；`fire_delay` 直接进拦截方程：

```
F(ψ, ϑ, τ) = p_b(τ; ψ, ϑ) − p̂(t0 + fire_delay + τ) = 0
```

**注意区分**：`predict_extra_delay`（图像到指令，≈15 ms）加在预测时刻 `future` 上；`fire_delay` 加在拦截时刻上。两者加错位置会重复补偿。

### S3 场上微调（定 `muzzle_offset` 与残差）

跑 10 发一组，看「命中点相对瞄准点的系统性偏移」：
- 恒定偏移 → 先查 `yaw_offset/pitch_offset` 与 `muzzle_offset`；
- 距离越远偏差越大 → 大概率 `c` 不准；
- 近距离大角度偏差 → `muzzle_offset` 非零（枪口不在旋转中心）。

---

## 6. 与电控（云台）的通信——两条链路能力不同

这是「这版自瞄会和电控通信」的关键：**弹道算出的角度能不能被云台用好，取决于下行链路**。

### 6.1 现状

| 链路 | 代码 | 帧内容 | 能否前馈 |
|---|---|---|---|
| **CAN**（步兵/无人机/哨兵） | [io/cboard.cpp:47-66](../io/cboard.cpp#L47-L66) | `control(1B) + shoot(1B) + yaw(int16) + pitch(int16) + horizon(int16)`，共 8 B | ❌ 只有目标角，无速度/加速度 |
| **串口**（MPC 机型） | [io/gimbal.cpp:99-118](../io/gimbal/gimbal.cpp#L99-L118) | `mode + yaw + yaw_vel + yaw_acc + pitch + pitch_vel + pitch_acc` | ✅ 可带速度/加速度前馈 |

对应主程序：
- [standard.cpp:77-79](../src/standard.cpp#L77-L79) → `cboard.send(command)`（CAN）
- [standard_mpc.cpp:80-82](../src/standard_mpc.cpp#L80-L82) → `gimbal.send(plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, ...)`（串口）

### 6.2 这意味着什么

1. **`Aimer` 升级（C2）对所有兵种生效**：它只输出目标角，CAN/串口都能用 → 收益立竿见影。
2. **`Planner` 升级（C3）只有在串口链路才发挥全部价值**：MPC 的 `yaw_vel/yaw_acc` 前馈依赖串口帧。
3. 若要让 CAN 机型也吃前馈，需要**改下位机协议**（两条路）：
   - 拆两帧：`0x0ff` 角度 + `0x0f1` 前馈；
   - 升级 CAN FD 或改走串口。
   
   > 这是跨组接口变更，建议本轮**只做视觉侧**，把 `Plan` 结构里的 vel/acc 留着，等电控就绪再接。

### 6.3 建议的落地顺序

```
Aimer（C2）—— 全链路收益，先上
   ↓
Planner 参考（C3）—— 串口机型先受益
   ↓
CAN 前馈扩帧 —— 需要电控配合，单独排期
```

---

## 7. 求解器接口与数值细节

### 7.1 接口

```cpp
namespace tools {

struct BallisticInput {
  double v0;                       // m/s，来自下位机弹速
  Eigen::Vector3d target;          // 世界系目标点（含预测）
  double fire_delay;               // s
  Eigen::Vector3d muzzle_offset;   // gimbal 系枪口偏心
  double yaw0, pitch0;             // 云台当前角（热启动）
  BallisticParams params;          // c, g, rk4_step, max_iter, max_fly_time
};

struct BallisticOutput {
  bool ok;
  double yaw, pitch;               // rad（pitch 抬头为正，与 tools::Trajectory 一致）
  double fly_time;                 // s
  int iterations;
};

BallisticOutput solve_ballistic(const BallisticInput &);
bool build_reference(const BallisticInput &, double dt, int horizon,
                     Eigen::Matrix<double, 4, Eigen::Dynamic> & out);  // MPC 参考
}
```

### 7.2 数值策略

| 项 | 取值 | 依据 |
|---|---|---|
| 积分 | RK4，`h=0.5 ms` | txt：`O(h⁴)`，生产用 3 步等效（端点差 < 2.2 mm） |
| Newton | 最多 8 次，判据 `‖F‖ < 1 mm` 且 `|Δτ| < 1 ms` | 平衡精度与耗时 |
| Jacobian | 解析灵敏度（18 维：`p, v, S_ψp, S_ψv, S_ϑp, S_ϑv` 一起 RK4） | txt §"云台角度迭代灵敏度" |
| 限幅 | `|Δψ|,|Δϑ| ≤ 0.1 rad`，`|Δτ| ≤ 0.05 s` | 防发散 |
| 时域 | 一阶 continuation：`q⁰ = q*_j + J⁻¹·∂F/∂s·Δs`；奇异/超界退回零阶 | txt §"从单次求解到完整时域" |
| 失败 | 返回 `ok=false` → 调用方回退 `tools::Trajectory` | 保证永不输出非法角度 |
| 耗时目标 | 单点 ≤ 0.2 ms，100 点参考 ≤ 6.5 ms（P99） | 对齐 txt benchmark |

### 7.3 接入点

| 位置 | 现状 | 改法 |
|---|---|---|
| [aimer.cpp:98](../tasks/auto_aim/aimer.cpp#L98)、[:127](../tasks/auto_aim/aimer.cpp#L127) | `tools::Trajectory(bullet_speed, d, z)` | `solve_ballistic`，失败回退 |
| [aimer.cpp:112-144](../tasks/auto_aim/aimer.cpp#L112-L144) | 飞行时间迭代 10 次 | 求解器内部已收敛，外层迭代可简化为 2 次（目标预测耦合） |
| [planner.cpp:179-203](../tasks/auto_aim/planner/planner.cpp#L179-L203) | 100 点逐点 `aim()` | `build_reference` + continuation |
| [shooter.cpp:19-41](../tasks/auto_aim/shooter.cpp#L19-L41) | 固定 3°/2° 容差 | 保持本轮不动（U4 另排） |

---

## 8. 验收（对应 C5）

```bash
# 单元测试：真空/阻力/边界/失败回退
./build/ballistic_solver_test

# 指标（对齐 txt benchmark）
#   e_miss     ≤ 2.2 mm   （与 4096 步 RK4 参考对比）
#   e_RK4      ≤ 2.2 mm   （3 步 vs 4096 步端点差）
#   单点耗时   P99 ≤ 0.2 ms
#   参考轨迹   P99 ≤ 6.5 ms（100 点）
#   失败率     0%（不可解工况必须回退，不许输出 NaN）
```

**回归**：
1. `bash tests/run_outpost_tests.sh --standalone` 全绿（`outpost_aimer_test` 对弹道不敏感，但必须不炸）；
2. 录像回放 `fire_ratio` 不劣化（≈0.33）；
3. 静态靶 3/5/8/10 m 预测误差 < 20 mm。

---

## 9. 风险

| 风险 | 影响 | 对策 |
|---|---|---|
| 实测 `c` 与物理估算差 2–5 倍 | 用物理值直接上场会系统性偏低 | **必须 S1 标定**；`resistance_coefficient` 覆盖项优先 |
| `fire_delay` 不稳定（摩擦轮掉速） | 拦截时刻抖动 | 先按均值 30 ms；若波动 >10 ms，改用「提前量区间」而非点值 |
| 求解器耗时挤占帧预算 | 帧率下降 | 单点限 0.2 ms + 失败回退；参考轨迹只给 MPC 机型 |
| CAN 无前馈，MPC 收益打折 | 升级体感不足 | 先上 Aimer；CAN 扩帧与电控单独排期 |
| 老配置缺 `ballistics` 段 | 行为变化（c 从 0 变物理值） | 缺省 `resistance_coefficient: 0` 时**按物理四项算**，但提供 `legacy_vacuum: true` 开关保证老配置零变化 |

---

## 10. 一句话给电控

> 视觉侧输出仍是 `yaw / pitch`（rad，抬头为正）与 `shoot`；串口机型额外给 `yaw_vel / yaw_acc / pitch_vel / pitch_acc` 前馈。C 系列升级不改变报文格式，只改变这两个角度的**计算精度**（新增阻力、开火延迟、枪口偏心）。CAN 机型若要吃到 MPC 前馈，需要扩帧，另行协商。
