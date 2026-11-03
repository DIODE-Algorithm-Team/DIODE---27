# 自瞄升级 & 可用目标清单（DIODE-27 × rmcs_auto_aim_v2 × 弹道解算算法）

> 用途：**审查与排期**。本文只做「可升级项 + 可复用资产」盘点，不含实现改动。
> 现状链路见 [outpost_flow_summary.md](outpost_flow_summary.md)，前哨站视觉方案见 [outpost_yolo_pose_plan.md](outpost_yolo_pose_plan.md)。
> 参考源码：`rmcs_auto_aim_v2/`、根目录 [readme.md](../readme.md)（上游说明，含 §4 轨迹视角理论）、[弹道解算及云台角度求解算法.txt](../弹道解算及云台角度求解算法.txt)。

> ⚠️ **2026-11-02 范围更新**：按「装甲板更新、去除灯条，这块不做」的约束，**U1（邻侧灯条真实端点）、U5 的灯条精修、YOLO-pose 的 kpt4/kpt5 已移出本轮**，执行范围以 [auto_aim_upgrade_plan.md](auto_aim_upgrade_plan.md) 为准。本文保留完整盘点用于历史审查。

---

## 0. 三分钟结论

1. **`sp_vision_25-main/` 曾是本工程的旧副本（现已在整理中归档到回收站），但文档有价值**：代码/资产与 DIODE-27 只差 5 个文件约 55 行，且全部是 DIODE 侧新增/修复（6 点 PnP 接线、前哨站 ω 退化保护、`kArmorThickness`、`world2pixel` const 化），模型同 hash → **无代码可挖**（证据见 §1.2）。但它带了一份 27 KB 的上游 `readme.md`，其中 **§4「轨迹视角下的自瞄理论」是 DIODE `Planner`（tinympc）的设计依据**，§4.4 明确写了开火延迟、预测时间偏移量、提前减速策略——**这篇文档要看**（提炼见 §1.3）。
2. **真正的升级来源是 `rmcs_auto_aim_v2`**：其中 4 个项目 DIODE 尚未具备且价值明确——①邻侧灯条**真实 2D 端点**检测、②角点亚像素精修、③带阻力的弹道解、④基于几何窗口的开火判定；另有时钟同步、匈牙利关联等工程化模块。
3. **弹道 txt 是"理论 + 仿真指标"，仓库里没有实现**：在源码/脚本范围内搜 `RK4`、`solveFrom`、`ballistic_reference_curve_benchmark` 均无命中（只有本文与 txt 自己提到），rmcs 侧只有简化版（半隐式欧拉 + 常数阻力 0.003）。→ 需**自研落地**，但它正好补上 DIODE 两个真实短板：弹道无阻力、MPC 参考轨迹无飞行时间前馈。

**投入产出排序（建议，已按新约束剔除灯条项）**：U2 带阻力弹道 > U6 MPC 参考角 > U4 开火窗口 > U3 开火延迟 > U13 延迟标定 > 其余。

---

## 1. 新增资产核查

### 1.1 总览

| 资产 | 位置 | 性质 | 对本项目的价值 |
|---|---|---|---|
| rmcs 自瞄（ROS2/C++20） | [rmcs_auto_aim_v2/README.md](../rmcs_auto_aim_v2/README.md) | 完整自瞄工程：检测/位姿/跟踪/火控 | **高**：多个 DIODE 缺失模块 |
| sp25 快照 | [readme.md](../readme.md)（已回迁到根目录） | 原副本与 DIODE-27 同源，已归档（回收站可恢复） | 代码 **≈0**；文档 **中高**（readme §4 轨迹视角理论） |
| 弹道算法文档 | [弹道解算及云台角度求解算法.txt](../弹道解算及云台角度求解算法.txt) | 396 行推导 + 仿真 benchmark | **高**：算法方向正确，**但无实现** |

> 注意：原 `sp_vision_25-main/` 副本（约 170 MB、833 个文件，含重复的 assets/io）已在 2026-11 整理时移入回收站；其中唯一有价值的上游文档已回迁为根目录 [readme.md](../readme.md)。

### 1.2 原副本判为「旧快照」的证据（归档前比对）

| 检查项 | 结果 |
|---|---|
| `tasks/ tools/ io/ src/ configs/` 文件级 hash 比对 | 仅 5 个文件不同：`aimer.cpp`(+18/−7)、`solver.cpp`(+24/−11)、`tracker.cpp`(+8/−0)、`outpost_const.hpp`(+4/−0)、`solver.hpp`(1 行 const) |
| 差异方向 | 全部是 DIODE 侧**新增**：`tracker.cpp:333-337` 的 6 点 PnP 接线、`aimer.cpp:58-65` 的 ω 退化保护、`outpost_const.hpp` 的 `kArmorThickness`、`world2pixel() const` |
| `assets/` 模型 | **完全相同**（无 yolo-pose、无新 onnx/xml） |
| 上游独有功能 | 无（`evaluate_outpost`、`solve_outpost_6pt`、yaw 三点插值在上游副本里**也已存在**，说明它就是同版本旧拷贝） |
| 上游独有文档/测试 | 仅 `readme.md`（27 KB 上游说明文档，含 §4 轨迹视角理论）；`autostart.sh` / `mpc_layout.xml` / `buff_layout.xml` / `LICENSE` / `.clang-format` 两边都有 |
| 结论 | 当「改造前基线」做 diff 参考可以，**不要**当作新代码来源；**只读它的 readme §4** |

### 1.3 上游 readme §4 的关键结论（与本次升级直接相关）

| 结论 | 对 DIODE 的含义 |
|---|---|
| 好自瞄的判据是**云台轨迹与射击轨迹的重合度**（`yaw_shoot(t) = yaw_target(t + t_fly)`），重合度决定 DPS 与击杀时间 | DIODE `Planner` 的 `Trajectory`（yaw/vel/pitch/vel 100 点参考）正是这套理论的实现，升级方向是**让参考轨迹更准**，而不是重写规划器 |
| 轨迹突变（切板）处"**提前减速**"过渡；方案一 = 隐式搜索 + TinyMPC（**国赛采用**），方案二 = 五次多项式显式搜索（仅仿真） | DIODE 已实现方案一（[planner.cpp](../tasks/auto_aim/planner/planner.cpp)），方案二可作为低依赖备选 |
| 开火延迟 `t_fire` 必须进决策：查 `t_fire` 时刻"射击轨迹 vs 规划轨迹"的误差，而非当前位置误差 | DIODE 用 `shoot_offset_` + `fire_thresh_` 近似；txt 的 `t_delay` 进拦截方程是更严格的做法（U3） |
| 预测时间 = 飞行时间 + 图像传输延迟 + 图像处理延迟 + 下位机通信/控制延迟；**后四项合成一个调试参数，实测约 15 ms** | 对上前哨站/高速目标，15 ms 对应 0.8π rad/s 下约 **2.1°** 偏差；DIODE 现有 `high/low_speed_delay_time` 应统一标定到该量级（sentry.yaml 为 26/10 ms） |
| 明确写出"图像传输延迟 = 图像时间戳对应**接收完成**时刻" | 印证 §U7：DIODE 的 `steady_clock::now()` 打戳方式会把传输延迟算进瞄准，须用相机硬件时间戳 + 同步模型 |

---

## 2. 自瞄升级清单（U 系列）

### 2.1 总览表

| # | 升级点 | 来源（rmcs / txt） | DIODE 落点 | 收益 | 风险 | 工作量 |
|---|---|---|---|---|---|---|
| **U1** | 邻侧灯条**真实 2D 端点**检测 | `module/detector/adjacency_lightbar.cpp` | `tracker.cpp:333-337` → `solver.cpp:94` | **高**：6 点 PnP 摆脱"纯几何闭环"，远距测距显著改善 | 中：ROI 阈值依赖光照 | 2–3 人日 |
| **U2** | 带空气阻力弹道（先移植 rmcs 简化版） | `fire_control/trajectory_solution.cpp` | `tools/trajectory.cpp` | 中高：远距 pitch/飞行时间更准 | 低：40 行，可保留解析解兜底 | 0.5–1 人日 |
| **U3** | 开火延迟进入拦截方程 | txt §3（`p_b(τ)=p̂(t_delay+τ)`） | `aimer.cpp:74-87` | 中：高速旋转前哨站更准 | 低 | 0.5 人日 |
| **U4** | 几何窗口开火判定 | `fire_control/shoot_evaluator.cpp` | `shooter.cpp:19-41` | 中高：替代"固定 3°/2° + 上次指令比较"，随距离/板朝向自适应 | 低 | 1 人日 |
| **U5** | 角点/灯条亚像素精修 | `utility/math/corners_optimizor.cpp` | `solver.cpp`（PnP 前）、detector 输出 | 中：PnP 距离与 yaw 更稳 | 低 | 1 人日 |
| **U6** | MPC 参考角 + 时域 continuation | txt §"从单次求解到完整时域"；readme §4.4 | `planner/planner.cpp:179-203` | **高**：参考轨迹含飞行时间前馈、切板处连续 | 中高：需自研求解器 | 4–6 人日（含 U2） |
| **U7** | 相机-下位机时钟同步 | `io/capturer/linear_sync_model.hpp`、`imu_snapshot_buffer.hpp` | `hikrobot.cpp:118`、`mindvision.cpp:100`、`standard.cpp:59` | 中高：消除"帧到达时刻当曝光时刻"的系统性延迟 | 中：依赖 SDK 硬件时间戳 | 3–4 人日 |
| **U8** | 枪口偏心/出射几何 | txt §"枪口变换" | `aimer.cpp:148`、`planner` 的 `aim()` | 中：近距离大角度偏差 | 中：需标定 `r_GM` | 2–3 人日 |
| **U9** | 多板关联（匈牙利） | `utility/math/hungarian.hpp` | `tracker.cpp` 选板逻辑 | 低中：多目标/多板场景更稳 | 低 | 1 人日 |
| **U10** | 运行期 TF 树 | `kernel/transform_tree.hpp`、`utility/tf` | `R_gimbal2imubody` 固定矩阵 | 低：工程化，短期无收益 | 中 | 3 人日 |
| **U11** | 马氏距离门控 | `utility/math/mahalanobis.hpp` | `target.cpp` NIS 已有 | **≈0**：DIODE 已有 NIS/NEES 检查 | — | 0（跳过） |
| **U12** | 摄像头捕获/回放框架 | `io/capturer/*`、`utility/image/recorder.hpp` | `io/*`、`tools/recorder.*` | 低：DIODE 已有同类 | — | 0（跳过） |
| **U13** | 预测时间偏移量统一标定 | readme §4.4（合计 ≈15 ms） | `configs/*.yaml` 的 `high/low_speed_delay_time` | 中：零代码，前哨站 0.8π 下 15 ms ≈ 2.1° | 低：需慢动作视频实测 | 0.5 人日 |

### 2.2 关键项详情

#### U1 邻侧灯条真实端点 → 6 点 PnP（最高优先级）

**现状缺陷**：[tracker.cpp:333-337](../tasks/auto_aim/tracker.cpp#L333-L337) 给 `solve_outpost_6pt` 传的是占位 `{-1,-1}`，[solver.cpp:136-145](../tasks/auto_aim/solver.cpp#L136-L145) 检测到负坐标后用 `world2pixel` 把**几何推算**的端点投回图像 —— 等于第 5/6 点没有任何真实观测，远距测距退化压不住（`outpost_flow_summary` §6.3 已记录）。

**rmcs 做法**（`adjacency_lightbar.cpp:36-120`）：用当前板 3D 位姿 + `NeighborBarSolution` 预测邻侧上/下灯条的**投影位置**，以其为中心开 ROI（`margin = max(12, 120/distance + 32)`），在 ROI 内做传统灯条检测 → 得到真实 `Lightbar2d`（upper/lower + is_right/is_upper），再交给 `optimize_corners` 精修，最后填进 `outpost_distance_optimizer.input`。

**移植改法**：新增 `tasks/auto_aim/neighbor_lightbar.{hpp,cpp}`（约 200 行：ROI 计算 + 灰度阈值 + 轮廓 + 灯条端点），在 [tracker.cpp:333](../tasks/auto_aim/tracker.cpp#L333) 之前调用，取到两点后替换 `{-1,-1}`。失败（遮挡/远距）返回 nullopt → 保持现有几何兜底，**零风险灰度**。

**验收**：人造 5 m/8 m/10 m 数据，6 点 PnP 测距误差相对 4 点下降 ≥50%（对齐 `outpost_migration` §6.2 `IT_6pt_pnp_distance`）。

#### U2 + U6 带阻力弹道 & MPC 参考角（最大的新增）

**现状**：
- [tools/trajectory.cpp:9-31](../tools/trajectory.cpp#L9-L31)：纯解析抛物线，`g=9.7833`，**无空气阻力**；
- [aimer.cpp:98](../tasks/auto_aim/aimer.cpp#L98)、[:127](../tasks/auto_aim/aimer.cpp#L127) 用它做"解一次 + 迭代飞行时间"（最多 10 次）；
- [planner.cpp:179-203](../tasks/auto_aim/planner/planner.cpp#L179-L203) 的 `get_trajectory`：对 100 个时域点**逐点重新调 `aim()`**，而 `aim()` 内部只按当前预测位置解弹道，**没有把飞行时间前馈进每个参考点**。

**txt 给出的方案**（`弹道解算及云台角度求解算法.txt`）：
- 弹丸 ODE：`ẋ = [v; −c|v|v_x; −c|v|v_y; −g−c|v|v_z]`，`c = ρC_Dπd²/(8m)`；
- 三未知量 `q = [ψ, ϑ, τ]`，残差 `F = p_b(τ;ψ,ϑ) − p̂(t_delay+τ)`，Newton 迭代 `JΔq = −F`；
- Jacobian 用**解析灵敏度**：扩维到 18 维状态 `[p, v, S_ψp, S_ψv, S_ϑp, S_ϑv]` 与弹道一起 RK4 积分；
- 时域推进用 continuation：零阶 `q⁽⁰⁾=q*_j`（当前 `solveFrom` 思路），一阶 `q⁽⁰⁾=q*_j + J⁻¹∂F/∂s·Δs`；奇异/超限/不收敛时逐级回退到完整 `solve()`；
- 仿真指标（文档自述）：250 时域点 × 4 板 × 1000 候选，平均 **4.716 ms**、P99 **6.333 ms**、最大参考脱靶 **2.137 mm**（4096 步 RK4 复算）。

**落地设计**（建议新增 `tools/ballistics.{hpp,cpp}`，保留 `tools::Trajectory` 作兜底）：

```cpp
struct BallisticInput  { double v0; Eigen::Vector3d target; double delay;
                         double yaw0, pitch0; };            // 云台当前角作初值
struct BallisticOutput { bool ok; double yaw, pitch, fly_time; int iterations; };

BallisticOutput solve_ballistic(const BallisticInput&);      // 单点：3 未知量 Newton
bool build_reference(const BallisticInput&, double dt, int horizon,
                     Eigen::Matrix<double,4,Eigen::Dynamic>& out); // 时域：continuation
```

- **接入点 1（Aimer）**：`aimer.cpp:98/127` 换成 `solve_ballistic`，`delay_time` 直接进方程（顺带完成 U3）；
- **接入点 2（Planner）**：`planner.cpp:179-203` 换成 `build_reference`，输出直接填 `Trajectory`（yaw, yaw_vel, pitch, pitch_vel）；
- **配置项**（新增，全部带缺省）：`bullet_mass`、`bullet_diameter`、`drag_coefficient`、`air_density`（或直接给合成系数 `c`）—— 建议先给 `c` 单参数，标定最简单；
- **验收**：自建 `tests/ballistic_solver_test.cpp`：网格工况（3/5/8/12/20 m × 静止/平移/旋转）× 4096 步 RK4 参考，输出 `e_miss`/`e_RK4`/耗时；阈值 `e_miss ≤ 2.2 mm`、P99 ≤ 6.5 ms；并与现解析解对比远距 pitch 差。

> 注意 1：txt 明确写了"**仅为理论参考，尚未实弹测试**"，因此上线必须保留解析解兜底 + 录像回放对比，不能直接替换。
> 注意 2：**不要重写规划器**。DIODE 的 `Planner` 已是 readme §4.4 方案一（隐式搜索 + TinyMPC）的落地，U6 只换它内部的参考生成（`get_trajectory`）；readme §4.4 的方案二（五次多项式显式搜索）可作为无第三方库依赖的备选，仅在 MPC 数值问题时考虑。
> 注意 3：`Planner::aim()` 现在每点只解"当前预测位置"的弹道，`target.predict()` 的调用顺序（[planner.cpp:183-200](../tasks/auto_aim/planner/planner.cpp#L183-L200)）依赖 `Target` 的增量预测语义，替换时要保持同样的单调时间推进，否则参考轨迹会错位。

#### U4 开火判定窗口

- 现状 [shooter.cpp:19-41](../tasks/auto_aim/shooter.cpp#L19-L41)：按距离在 3°/2° 两档固定容差，且比较的是"上一次 command"与"云台角"，前哨站转速高时 `last_command` 语义偏弱。
- rmcs `shoot_evaluator.cpp:69-130`：把容差换算成**笛卡尔窗**（横向 `yaw_tolerance`、纵向 `pitch_tolerance`），再乘 `cos(板心夹角)` 做透视缩放，距离 >5 m 时钳制不再收窄，并做 `finite` 防御。
- 收益：前哨站三块板在斜侧时"窗口宽度"物理正确，减少"看着对准了其实擦边"的误开火。

#### U7 时钟同步（与之前"图传/延迟"问题同源）

- 现状：三个相机实现都用**帧到达时刻** `steady_clock::now()` 打时间戳（[hikrobot.cpp:118](../io/hikrobot/hikrobot.cpp#L118)、[mindvision.cpp:100](../io/mindvision/mindvision.cpp#L100)、[usbcamera.cpp:149](../io/usbcamera/usbcamera.cpp#L149)），[standard.cpp:59](../src/standard.cpp#L59) 用同一时刻的 IMU 解算。
- rmcs：`hikcamera` 取曝光时间戳，`LinearSyncModel` 用相机时间戳 ↔ 板子时间戳做最小二乘线性拟合（含残差阈值、协方差），`ImuSnapshotBuffer` 按时间戳 pop 最近 IMU 快照。
- 收益：把"帧到达延迟"变成可观测的固定偏差，对高速目标（前哨站 0.8π）与远距 PnP 都直接有效。

---

## 3. 可使用的目标清单

### 3.1 可瞄准目标类型（能力矩阵）

| 目标 | 现有支持 | 关键实现 | 本次升级影响 |
|---|---|---|---|
| 步兵 / 英雄 / 工程（4 板，大/小） | 支持 | 11 维 `Target` + `choose_aim_point` | U2/U3/U4/U6 均受益（弹道、开火、MPC） |
| 哨兵（4 板） | 支持 | 同上 | 同上 |
| **前哨站（3 板，小装甲板）** | 支持 | `OutpostTarget` 5 维 + 6 点 PnP + 动态窗口 | **U1/U2/U3/U4 主战场** |
| 基地（3 板，r=0.3205） | 支持 | `Target` 分支 | 弹道类升级受益 |
| 能量机关（小/大符） | 支持（独立链路） | `tasks/auto_buff` | 不受影响（U 系列不改 buff） |
| 无人机前哨站档 | 支持 | `src/uav.cpp:78`（`outpost(4)`≡`auto_aim(1)`） | 同上 |

### 3.2 可直接复用 / 需改造 / 不可用

| 资产 | 结论 | 说明 |
|---|---|---|
| `AdjacencyLightbarFinder` | **需改造（U1）** | 算法可用，类型要换成 DIODE `Armor`；PIMPL 与 ROS 类型需剥离 |
| `TrajectorySolution` | **可直接复用（U2）** | 84 行、无外部依赖，重命名为 `tools::Trajectory` 的 drag 变体即可；`kAirResistanceCoefficient=0.003`、`kGravity=9.81` 需做成配置 |
| `ShootEvaluator` | **需改造（U4）** | `Point3d`/`Command` 换成 `Eigen::Vector3d`/`io::Command`，逻辑可整体照搬 |
| `optimize_corners` | **可直接复用（U5）** | 两个自由函数，签名 `(const cv::Mat&, Armor2ds&)` / `(const cv::Mat&, Lightbar2d&)`，换成 DIODE `Armor::points` 即可 |
| `LinearSyncModel` / `ImuSnapshotBuffer` | **需改造（U7）** | 依赖 `rmcs_msgs::ImuSnapshot` 与相机硬件时间戳；海康 SDK 有 `stFrameInfo.nDevTimeStamp` 可用 |
| `NeighborBarSolution` / `OutpostSolution` | **已具备** | DIODE [outpost_geometry.hpp](../tasks/auto_aim/outpost_geometry.hpp) 已移植 |
| `outpost_distance_optimizer` / 6 点 PnP | **已具备** | DIODE [solver.cpp:94-199](../tasks/auto_aim/solver.cpp#L94-L199) |
| yaw 优化 + 三点插值 | **已具备** | DIODE [solver.cpp:307-351](../tasks/auto_aim/solver.cpp#L307-L351)，与 rmcs `yaw_optimizer.cpp` 等价（同为 ±70°/1°/二次插值） |
| `OutpostModel` / `HeightBuffer` / 切板 / 折算 | **已具备** | 见 `outpost_flow_summary` |
| `FireController` 动态攻击窗口 | **已具备** | DIODE `Aimer::evaluate_outpost` / `get_attack_window` |
| `EKF`（Joseph form） | **已具备** | DIODE `tools::ExtendedKalmanFilter` |
| `RuneModel` / `virtual_rune` / `rune_energy_fitter` | **不可用** | 与本项目打符链路不兼容，且不在升级范围 |
| ROS2 / rclcpp / PIMPL / coroutine / `feishu.hpp` / streaming | **不可用** | 架构不兼容（单进程标准 C++ + OpenVINO） |
| `sp_vision_25-main/*` | **不可用（重复）** | 旧副本，见 §1.2 |

---

## 4. 优先级与执行阶段

依赖关系：`U2 → U6`、`U13`（纯配置，可最先做）、`U3`（依赖 U2 的 `delay` 入方程）、`U4/U5` 独立；`U1` 已移出本轮。

| 阶段 | 内容 | 交付物 | 验收 |
|---|---|---|---|
| **P0**（先做，2 周内） | U13 + U2 + U3 | 配置标定、`tools/ballistics.*`、aimer 接线 | 前哨站 4 个既有测试全绿；新增 `ballistic_solver_test` |
| **P1**（下阶段） | U4 + U6 + U5（仅角点） | `Shooter` 窗口化、`build_reference` 接 Planner、角点精修 | 录像回放 `fire_ratio`/跟踪稳定度不劣化；MPC 参考轨迹连续性检查；单帧耗时 ≤ 6.5 ms |
| **P2**（有余力） | U7 + U8 + U9 + U10 | 时钟同步、枪口偏心、匈牙利、TF 树 | 需实车标定/长期回归，单独排期 |
| **跳过** | U11、U12、Rune、ROS2 | — | — |

**每阶段共同门槛**（沿用现有约定）：
1. `bash tests/run_outpost_tests.sh --standalone` 全绿；
2. 地面兵种录像回归（`assets/demo`）无差异；
3. 前哨站录像回放的 `fire_ratio`（健康值 ≈0.33）与 `no_control` 不劣化；
4. 改动只落在 `armor.name == outpost` 分支或新增文件，不改 5 维 EKF 状态定义。

---

## 5. 需要你决策的 5 个问题（审查前请确认）

1. **U1 是否做**？它是唯一能实质提升远距测距的项，但需要新增约 200 行传统 CV，且 ROI 阈值对场地光照敏感（前哨站多为反光/逆光）。
2. **弹道参数怎么给**？三个方案：(a) 只暴露合成系数 `c`（标定最省事）；(b) 暴露 `m/d/C_D/ρ` 四项（物理可解释）；(c) 直接沿用 rmcs 的 `0.003` 常数（零标定、精度一般）。
3. **是否上 MPC 参考（U6）**？DIODE 已有 `Planner` + tinympc，但当前 `standard`（非 mpc）走的是 `Aimer`，两条链路要一起升级还是先只升 `Aimer`？
4. **时钟同步（U7）依赖相机硬件时间戳**：你的相机是海康（有 `nDevTimeStamp`）还是 UVC？UVC 无硬件时间戳，U7 只能做"软件延迟补偿"。
5. **`delay_time` 取值**：txt 把开火延迟作为一等公民，现有配置只有 `high/low_speed_delay_time`（0.0–0.066 s）。需要实测摩擦轮发弹延迟吗？

---

## 6. 审查用检查清单

- [ ] 确认 `sp_vision_25-main` 不再作为方案来源（避免重复审查）
- [ ] 确认 U 系列只改前哨站分支 / 新增文件，不动普通兵种链路
- [ ] 确认 `tools::Trajectory` 保留为兜底（txt 自述未实弹验证）
- [ ] 确认弹道参数 `c`（或 `m/d/C_D/ρ`）的来源与标定方法
- [ ] 确认新增配置键都有缺省值（否则 `Tracker`/`Aimer` 构造即抛异常，历史上 `outpost_max_temp_lost_count` 已踩过）
- [ ] 确认验收指标与现有 4 个 outpost 测试不冲突
