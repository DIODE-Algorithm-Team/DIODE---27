# 前哨站方案迁移技术文档 (rmcs_auto_aim_v2 → sp_vision_25)

> **注**：本文是设计/迁移计划（写作时实现尚未落地）。代码已按 PR-1..PR-6 实施完成，
> 实际流程、代码位置、使用方法与参数说明见 [outpost_flow_summary.md](outpost_flow_summary.md)。

> 本文档基于对 `D:\RM\rmcs_auto_aim_v2` 与 `d:\RM\sp_vision_25-main` 双方源码的逐文件分析，给出 rmcs 方案的核心算法提取、分阶段实施计划、可复用模块清单、接口名称映射表与测试设计，作为后续编码阶段的依据。

---

## 一、rmcs 前哨站核心算法分析

### 1.1 状态量与观测量（5维状态 / 4维观测）

rmcs 的 `OutpostModel` 位于 `src/module/tracker/model/outpost.cpp`，使用极简状态量：

| 维度 | 含义 | 单位 |
|---|---|---|
| 0 / 1 / 2 | 旋转中心在世界系下的 x / y / z | m |
| 3 | 转速 ω | rad/s |
| 4 | 参考板 yaw 角（约定 center→armor 方向） | rad |

观测量 `[center.x, center.y, center.z, armor_yaw]` 直接来自 PnP 解算后的 3D 信息（板中心 + yaw），与 SP 当前的 4 维 ypd+a 观测（球坐标 + 角度）形成对比。这是 rmcs 方案的底层差异：**前哨站几何完全固定，3D 观测噪声可由距离优化压制到可接受水平**，因此直接吃 3D 观测比走 2D 像素更高效。

### 1.2 几何先验常量（`src/utility/robot/constant.hpp`）

```cpp
kOutpostRadius            = 0.275;   // 旋转半径 (SP: 0.2765)
kOutpostArmorHeightStep  = 0.102;   // 三块板高度差
kPredictedOutpostArmorPitch = -15°;
kOutpostAngularSpeed     = 0.8π;    // 仅用于 test 真值
kAngleStep               = 2π/3;    // 三块板 120° 夹角
```

三块板偏移表 `kOffsetTable`（按高 → 中 → 低排列）：

```cpp
{ 0°,    0 step },   // 板 0 - 最高 (参考板)
{ 120°, -1 step },   // 板 1 - 中
{ 240°, -2 step }    // 板 2 - 最低
```

### 1.3 切板检测与方向锁定（关键技术点 ①）

`OutpostModel::Impl::correct()` L352-393：

- **切板判定**：相邻帧观测 yaw 跳变 `|Δyaw| > plate_switch_yaw_min`（默认 50°）即判为切板。三块板夹角 120°，建议阈值 50°~80°。
- **方向锁定**：用 `pending_sign` + `sign_evidence_count` 状态机，**连续两次同方向切板** 才将 `rotation_sign` 锁定（`kSignConfirmCount = 2`）。锁定前用单帧 sign，锁定后稳定方向，抗视觉噪声。
- **位置验证门控**：切板检测前先做 `xy_error` 检查，距离 >5m 时 0.20m，否则 0.10m，超限直接丢弃该帧（不更新 EKF），避免噪声污染。

### 1.4 板序号在线辨识（关键技术点 ② - HeightBuffer）

`HeightBuffer`（L98-175）是 rmcs 方案最具创新性的设计：

- **三槽位滑动缓冲**：`std::array<double, 3> data`，初值 NaN，`factor=0.1` 指数加权。
- **NaN 自动跳过**：`is_lower/is_upper` 在参与比较的任一值为 NaN 时返回 true，使当前板 level 仅由已观测邻居确定。
- **level_at() 比较逻辑**：返回当前板序号 0/1/2（高/中/低）。
- **三块板齐后纠正绝对序号**：`abs_ref_index = (height_buff.level() - rel_see_index + 3) % 3`，然后 `clean_oldest()` 清理最旧板。
- **未观测板自动跳过**：仅观测两块板时也能给出已观测板间正确相对高度，足以支持 EKF 迭代；待第三块板观测到后整组序号完全确定。

### 1.5 参考板坐标系折算（关键技术点 ③）

EKF state 跟踪的是构造时第一块观测板（参考板）。切板后将当前板观测**折算回参考板坐标系**：

```cpp
// rel_see_index ∈ {-1, 0, +1}：当前板相对参考板的偏移
if (rel_see_index != 0) {
    delta_yaw    = in_right ? +2π/3 : -2π/3;
    delta_height = outpost_relative_height(in_right, in_upper);
}
observation[2] -= delta_height;             // 高度折算
observation[3] = normalize_angle(observation[3] - delta_yaw);  // yaw 折算
```

`outpost_relative_height` 的四象限表（`src/utility/math/outpost.cpp` L38-48）：

| in_right | in_upper | delta_height |
|---|---|---|
| true | true  | +2 step |
| true | false | -1 step |
| false| true  | +1 step |
| false| false | -2 step |

更新后 `last_observation` 反补偿回当前板坐标系，使下一帧切板检测仍正确。

### 1.6 6 点 PnP 距离优化器（核心技术增益）

`src/utility/math/solve_pnp/outpost_distance_optimizer.cpp`：

- **构造 6 个对应点**：4 个装甲板角点 + 2 个**邻侧板灯条端点**（上灯条顶/底 或 下灯条顶/底，由 `is_upper` 选）。
- **邻侧板位置由几何先验推算**：`NeighborBarSolution` 用当前板 `translation + orientation` + 装甲板厚度 `armor_thickness` + 120° 旋转几何约束算出邻侧板的位姿，再取其灯条端点作为额外约束点。
- **优化方法**：`cv::solvePnP(..., SOLVEPNP_ITERATIVE)`，使用初始 rvec/tvec 作为热启动。
- **效果**：远距离下 PnP 4 点法测距退化，引入邻侧灯条端点后**显著压制测距误差**，这是前哨站链路相对地面兵种的核心增益。
- **所需输入**：4 板角点、邻侧上/下灯条 2D 端点、`is_right`、`is_upper`、`armor_thickness`。

### 1.7 yaw 优化（重投影 + 三点插值）

`pnp_solution.cpp::RobustPnpSolution::solve()` L171-267：

- ±60° 范围 1° 步长搜重投影最小 yaw（与 SP 的 `optimize_yaw` ±70° / 1° 接近）。
- **额外增益**：搜到最优 yaw 后，取左右各 1° 的误差做**三点二次插值**，亚度级精化 yaw：
  ```
  interpolated_yaw = optimized_yaw + step * 0.5 * (err_low - err_up) / (err_low - 2*err_opt + err_up)
  ```
- 评估插值误差更小时才接受。

### 1.8 EKF 数学形式（标准 Joseph form）

`OutpostModel::Impl` L207-270：

- **predict**：`angle = normalize(angle + ω·dt)`，状态雅可比 `F(4,3)=dt`，`P_pre = F·P·Fᵀ + Q`。
- **correct**：观测雅可比 `H = [I₃, 0; 0, 0, 1]`（仅取 x,y,z,angle），标准卡尔曼增益 `K = P·Hᵀ·(H·P·Hᵀ+R)⁻¹`。
- **协方差更新**：Joseph form `(I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ`，强制对称化。
- **发散检测**：协方差 >150、|state|>15m、|ω|>10π、NaN/Inf、超时 500ms 任一触发发散。
- **收敛检测**：update_count≥5 且 `cov(0,0),cov(1,1)<0.01, cov(4,4)<0.01`。

### 1.9 开火控制（fire_control）

`FireController::aim`（L310-443）：

- **Trackable 抽象**：通过 `get_direction/get_aimpoints/get_rotation_speed/jump_into/clone` 统一前哨站/机器人/能量机关符接口。
- **动态攻击窗口**：`get_attack_window` 基于锯齿波建模 + 云台最大速度/加速度约束，目标角速度超 `degraded_angle_speed`（默认 12 rad/s）时退化为瞄中心。
- **施密特滞回**：`enter_window = half·(1-hyst)`，`exit_window = half·(1+hyst)`，避免边界震荡。
- **预瞄**：窗口内无板时瞄来板一侧边界，减少云台跟踪量。
- **退化为瞄中心时**：用 `armor_on_ray`（横向偏移不超过板半宽）决定是否发弹。

---

## 二、SP 项目现状与已知缺陷

### 2.1 SP 现有前哨站实现位置

| 文件 | 关键位置 | 现状 |
|---|---|---|
| `tasks/auto_aim/target.cpp` | L97-103（v1/v2）、L131-133（ω钳制）、L248-251（构造）、L258-261（收敛）、L267-278（h_armor_xyz） | 11维状态 `[cx,vx,cy,vy,cz,vz,θ,ω,r,l,h]`，前哨站 `armor_num=3, r=0.2765, l/h方差=0` |
| `tasks/auto_aim/tracker.cpp` | L45-50（顶部板过滤注释）、L220-224（temp_lost=75）、L280-290（多板循环 update） | **缺陷**：update_target 把所有同 name+type 板循环送 EKF，应只取最匹配一块 |
| `tasks/auto_aim/aimer.cpp` | L144-208 choose_aim_point | **缺陷**：`!jumped` 时死跟 id=0（L150） |
| `tasks/auto_aim/solver.cpp` | L55-88（4点IPPE）、L96（pitch=-15°）、L196-218（optimize_yaw）、L129-194（oupost_reprojection_error） | **没有 6 点 PnP 距离优化** |
| `tasks/auto_aim/shooter.cpp` | L19-41 开火判断 | 容差分档近 3° / 远 2° |

### 2.2 SP 命名规范（迁移必须遵循）

| 维度 | 规范 |
|---|---|
| 命名空间 | `namespace auto_aim` |
| 类名 | PascalCase：`Tracker` / `Solver` / `Aimer` / `Target` |
| 方法名 | snake_case：`track` / `set_target` / `update_target` / `choose_aim_point` |
| 成员变量 | 带下划线后缀：`target_` / `state_` / `ekf_` |
| 枚举 | `ArmorName` / `ArmorType` / `ArmorPriority` |
| 常量 | UPPER_CASE |
| 配置 | yaml-cpp 读取 |
| 角度规约函数 | `tools::limit_rad`（对应 rmcs `normalize_angle`） |
| EKF | `tools::ExtendedKalmanFilter`（非模板类，VectorXd/MatrixXd 动态维度） |

### 2.3 SP 数据模型差异

SP 的 `Armor` 结构使用 `xyz_in_world` / `ypr_in_world` / `ypd_in_world`（Eigen::Vector3d），而非 rmcs 的 `Armor3d{translation, orientation}`（四元数+平移）。迁移时需要做坐标系约定转换：

- rmcs `armor_yaw = atan2(-armor2center.y, -armor2center.x)`（center→armor 方向）
- SP `armor.ypr_in_world[0]` 是装甲板自身朝向 yaw（armor→center 方向相反侧，需对照确认）

---

## 三、分阶段实施计划

### 阶段 1：基础工具层移植（低风险，无外部依赖）

**目标**：把 rmcs 中独立的纯算法模块移植为 SP 风格的工具函数，不触碰现有 Target/Tracker 主流程。

| 任务 | 来源 | 输出位置 | 说明 |
|---|---|---|---|
| 1.1 HeightBuffer 板序号辨识 | `outpost.cpp::HeightBuffer` L98-175 | `tasks/auto_aim/outpost_helper.hpp` | 改 `std::array<double,3>` + `factor`，命名 `class HeightBuffer` |
| 1.2 切板方向确认状态机 | `outpost.cpp` L189-194 + L363-375 | 同上文件 `struct PlateSwitchDetector` | 抽出 pending_sign/sign_evidence_count/rotation_sign 三态机 |
| 1.3 参考板坐标系折算 | `outpost.cpp` L397-406 + `outpost_relative_height` | 同上 `outpost_relative_height` 自由函数 | 常量 `kHeightStep`、`kAngleStep`、`kRadius` 放 `namespace auto_aim::outpost_const` |
| 1.4 几何常量集中 | `rmcs/src/utility/robot/constant.hpp` | `tasks/auto_aim/outpost_const.hpp` | SP 化：`kOutpostRadius=0.2765`（沿用 SP 旧值）、`kHeightStep=0.102`、`kPredictedPitch=-15°`、`kPlateSwitchYawMin=50°`、`kSignConfirmCount=2` |

**验收**：单元测试构造人造切板序列，验证 HeightBuffer level() 输出、方向锁定时序、折算公式正确性。

### 阶段 2：6 点 PnP 距离优化器移植（中风险）

**目标**：在 SP `Solver` 类中新增 `solve_outpost_6pt` 方法，远距离下显著提升测距精度。

| 任务 | 来源 | 输出 | 依赖 |
|---|---|---|---|
| 2.1 邻侧板几何推算 | `utility/math/outpost.cpp::OutpostSolution/NeighborBarSolution` | `tasks/auto_aim/outpost_geometry.hpp/cpp` | SP `Armor` 的 xyz/ypr → 推算邻侧板位姿 |
| 2.2 6 点 PnP 主流程 | `outpost_distance_optimizer.cpp` | `Solver::solve_outpost_6pt(Armor&, is_right, is_upper, upper_pt, lower_pt, thickness)` | OpenCV `solvePnP ITERATIVE`，热启动用 4 点 IPPE 结果 |
| 2.3 邻侧灯条 2D 端点获取 | 需 detector 输出邻侧灯条 | 与 SP 现有 `Lightbar` 结构对齐 | 若 SP detector 不输出邻侧灯条，回退到用 SP 现有 4 点法 + 阶段 1.3 几何约束算虚拟灯条端点 |
| 2.4 yaw 重投影 + 三点插值 | `pnp_solution.cpp` L171-267 | 升级 `Solver::optimize_yaw` 增加插值步 | 与 SP 旧 `optimize_yaw` 融合，不替换 |

**风险点**：邻侧灯条 2D 端点是否能在 SP detector 链路拿到。若拿不到，可降级为只用 4 装甲板角点 + 通过几何约束**生成的虚拟邻侧板灯条 3D 点**（仍能起到几何约束作用）。

**验收**：人造 5m / 8m / 10m 远距离测试数据，比较 4 点 vs 6 点测距误差。

### 阶段 3：OutpostModel 状态量精简（中风险，核心改动）

**目标**：在 SP `Target` 类中加前哨站特化分支，采用 rmcs 的 5 维状态 + 4 维 3D 观测。**保留** SP 现有 11 维 Target 用于普通车，前哨站走新分支。

**两种实施策略（择一）**：

#### 策略 A（推荐）：新建独立 `OutpostTarget` 类
- 新文件 `tasks/auto_aim/outpost_target.hpp/cpp`，类 `OutpostTarget`，独立于 `Target`。
- 内部 5 维 EKF，方法名对齐 SP 规范：`predict(t)` / `update(armor)` / `ekf_x()` / `armor_xyza_list()` / `diverged()` / `convergened()`。
- 在 `Tracker::set_target` 中：`armor.name == ArmorName::outpost` 时构造 `OutpostTarget`，否则构造 `Target`。
- 用 `std::variant<Target, OutpostTarget>` 或基类指针在 Tracker 中统一管理。
- **优点**：隔离风险，不动现有 11 维 EKF，回归零负担。
- **缺点**：需改 Tracker/Aimer 接口支持多态，工程量稍大。

#### 策略 B：在 `Target` 类内加前哨站分支
- 改动 `Target` 类构造函数 / predict / update / h_armor_xyz / h_jacobian，根据 `name==outpost` 切换状态量维度与观测模型。
- **优点**：改动集中、不引入新文件。
- **缺点**：Target 类会变臃肿，且 SP 现有 EKF 是 `tools::ExtendedKalmanFilter`（VectorXd 动态维度），切 5 维后需保证现有 11 维路径不受影响。

**建议**：策略 A。原因：rmcs 5 维状态与 SP 11 维状态差异过大，强行共用一类会让代码维护成本飙升，且 SP 项目"最小改动"原则更适合隔离方式。

### 阶段 4：Tracker 接入 + 修缺陷（中风险）

| 任务 | 现有缺陷位置 | 修复方案 |
|---|---|---|
| 4.1 修 `tracker.cpp:280` 多板循环 update | `update_target` L280-290 | 前哨站路径下只取最匹配一块板送 EKF（按 `|Δ朝向|+|Δ方位|` 最小），普通车可保留多板循环 |
| 4.2 修 `aimer.cpp:150` !jumped 死跟 id=0 | `choose_aim_point` | 移除 `jumped` 依赖，始终选最正对枪管的板（按来板/去板角度阈值 coming=70°/leaving=30°） |
| 4.3 切板检测接入 | rmcs 切板逻辑 | 用阶段 1.2 的 `PlateSwitchDetector`，在 `OutpostTarget::update` 内部检测切板后调用阶段 1.3 折算 |
| 4.4 板序号辨识接入 | rmcs HeightBuffer | `OutpostTarget` 内部维护 `HeightBuffer`，三块板齐后纠正 `abs_ref_index` |
| 4.5 图像中心硬编码修复 | `tracker.cpp:54,119` `1440/2,1080/2` | 改为从 Solver/配置读取 camera_matrix 的 cx/cy |

### 阶段 5：Aimer 改造（中风险）

- 复用 SP 现有时间补偿 + 弹道迭代（aimer.cpp L45-116，质量已够用）。
- `choose_aim_point` 改造：参考 rmcs `evaluate_armor` + `boundary_point`，引入：
  - 动态攻击窗口 `get_attack_window`（前哨站 ω 固定 0.8π 不退化，但保留接口）
  - 施密特滞回 `enter_window/exit_window`（hyst=0.2）
  - 预瞄逻辑（窗口内无板时瞄来板一侧边界）
- 保留 SP 的 coming_angle=70°/leaving_angle=30° 作为 fallback。

### 阶段 6：测试与验证

详见第六章。

---

## 四、可复用模块清单与兼容性评估

### 4.1 可直接复用（命名替换后即可移植）

| 模块 | rmcs 来源 | 兼容性 | 替换要点 |
|---|---|---|---|
| HeightBuffer | `outpost.cpp::HeightBuffer` | ✅ 完全独立，无外部依赖 | 仅依赖 `<array>` + `<cmath>`，命名空间改 `auto_aim`，方法名 snake_case |
| 切板方向确认三态机 | `outpost.cpp` L189-194 + L363-375 | ✅ 独立 | 抽出为 `PlateSwitchDetector` 结构体，接口 `update(sign) -> int` 返回锁定方向 |
| `outpost_relative_height` | `utility/math/outpost.cpp` L38-48 | ✅ 纯函数 | 直接复制，常量改用 SP 的 `kHeightStep` |
| 参考板坐标系折算逻辑 | `outpost.cpp` L397-406 | ✅ 独立 | 包装为 `transform_to_reference(observation, rel_see_index)` |
| 观测位置验证 | `outpost.cpp` L337-345 | ✅ 独立 | 包装为 `is_observation_valid(obs, last_obs, distance)` |
| 三点二次插值 | `pnp_solution.cpp` L246-263 | ✅ 独立 | 与 SP `optimize_yaw` 融合，新增插值步 |
| EKF Joseph form 协方差更新 | `ekf.hpp` L147-152 + `outpost.cpp::a_posteriori_update` | ✅ 算法独立 | SP `tools::ExtendedKalmanFilter` 已是 Joseph form（看源码确认），无需替换 |

### 4.2 需适配（架构差异需改造）

| 模块 | 差异 | 适配方案 |
|---|---|---|
| `OutpostModel` 类 | rmcs 用 PIMPL + Armor3d（Quaterniond+Translation），SP 用 Target+Armor（xyz_in_world+ypr_in_world） | 新建 `OutpostTarget` 类，输入输出对齐 SP 的 Armor 结构 |
| EKF 模板 `EKF<5,4>` | rmcs 是模板类静态维度，SP `tools::ExtendedKalmanFilter` 是动态维度 | 用 SP 现有 EKF 类，状态量切 5 维，观测切 4 维，predict/update 传 5×5/4×5 矩阵 |
| `Armor3d` 数据结构 | rmcs 用四元数+平移，SP 用欧拉角+xyz | 在 `OutpostTarget::update` 内部由 `armor.xyz_in_world` + `armor.ypr_in_world` 构造观测向量 `[cx, cy, cz, yaw]`，其中 yaw 按 rmcs 约定取 `atan2(-armor2center.y, -armor2center.x)` 等价值（需对照 SP 现有约定确认符号） |
| 6 点 PnP 距离优化器 | rmcs 用 ROS 系/OpenCV 系坐标系转换；SP 用 gimbal/world 系 | 移除 `ros2opencv_position/rotation` 调用，全部用 SP 的 `R_gimbal2world_` + `R_camera2gimbal_` 链路 |
| `NeighborBarSolution` | rmcs 几何推算依赖四元数旋转 | 改造为：用 SP `Armor` 的 `ypr_in_world` 构造旋转矩阵，按 120° 旋转 + height_step 平移得到邻侧板位姿 |

### 4.3 不复用（架构不兼容）

| 模块 | 原因 |
|---|---|
| ROS2 / rclcpp 框架 | SP 是单进程标准 C++ + OpenVINO |
| PIMPL 模式（`RMCS_PIMPL_DEFINITION`） | SP 直接类，不藏实现 |
| `Trackable` 抽象类 + `Ins<State>` 模板 | SP 用 `std::list<Target>` 值类型，无多态 |
| `Repeat/Repeat::Action` 时序状态机 | 仅能量机关符需要，前哨站不用 |
| `RotationEstimator`（KLT 光流） | 与 SP 现有 detector 视觉前端冲突，不引入 |
| `FireController` 整体 | SP 现有 `Aimer` + `Shooter` 已覆盖；仅移植 `evaluate_armor` 的滞回 / 预瞄思想到 `choose_aim_point` |
| `RuneModel` | 不在本次前哨站迁移范围 |
| `Serializable` 配置序列化 | SP 用 yaml-cpp 直接读 |

### 4.4 接口名称映射表（rmcs → SP）

| rmcs 接口 | SP 替换后名称 | 备注 |
|---|---|---|
| `namespace rmcs` / `rmcs::util` | `namespace auto_aim` | 全局命名空间 |
| `OutpostModel` 类 | `OutpostTarget` 类 | 与 SP `Target` 并列 |
| `OutpostModel::Config` 结构体 | YAML 节点直接读 | 不引入新 Config 结构体，对齐 SP `Aimer` 模式 |
| `OutpostModel(armor)` 构造 | `OutpostTarget(armor, t, radius, armor_num, P0_dig)` | 对齐 SP `Target` 签名风格 |
| `predict(double dt)` | `predict(double dt)` / `predict(time_point t)` | 与 SP `Target::predict` 同名 |
| `correct(const Armor3d&)` | `update(const Armor&)` | SP 用 `update`，且 `Armor` 替换 `Armor3d` |
| `state()` 返回 `State{x,y,z,rotation_speed,rotation_angle,index}` | `ekf_x()` 返回 `Eigen::VectorXd` | 对齐 SP `Target::ekf_x` |
| `full()` 返回 `std::array<Armor3d,3>` | `armor_xyza_list()` 返回 `std::vector<Eigen::Vector4d>` | 对齐 SP `Target::armor_xyza_list` |
| `current()` 返回当前板 `Armor3d` | 内部维护，不对外暴露 | SP 用 `armor_xyza_list` 即可 |
| `converge()` | `convergened()` | 对齐 SP 拼写（已存在拼写） |
| `diverged()` | `diverged()` | 同名 |
| `normalize_angle(x)` | `tools::limit_rad(x)` | 全文替换 |
| `deg2rad(x)` | `x / 57.3` 或 `x * CV_PI / 180` | 对齐 SP 现有风格 |
| `Armor3d.translation` | `Armor.xyz_in_world` | 类型 `Eigen::Vector3d` 兼容 |
| `Armor3d.orientation` (Quaterniond) | `Armor.ypr_in_world` (Vector3d) | 需在 `OutpostTarget::update` 内部转换 |
| `DeviceId::OUTPOST` | `ArmorName::outpost` | 全文替换 |
| `kOutpostRadius` | `outpost_const::kRadius` (0.2765 沿用 SP) | 放 `outpost_const.hpp` |
| `kOutpostArmorHeightStep` | `outpost_const::kHeightStep` (0.102) | 同上 |
| `kPredictedOutpostArmorPitch` | `outpost_const::kPredictedPitch` (-15°) | 同上 |
| `Clock::now()` | `std::chrono::steady_clock::now()` | 已对齐 |
| `Timestamp` / `TimePoint` | `std::chrono::steady_clock::time_point` | 已对齐 |

---

## 五、与 SP 整体架构的兼容性

### 5.1 依赖关系

| 依赖 | SP 现有 | rmcs | 兼容性 |
|---|---|---|---|
| OpenCV | ✅ | ✅ | 完全兼容，6 点 PnP 用 `cv::solvePnP` |
| Eigen | ✅ | ✅ | 完全兼容 |
| yaml-cpp | ✅ | ✅ | 完全兼容 |
| OpenVINO | ✅ | ❌（用 ONNX） | 不冲突，rmcs 不引入 OpenVINO |
| ROS2 | ❌ | ✅ | **不引入**，移除所有 rclcpp 依赖 |
| C++20 concepts/generators | 部分 | ✅（用 `<generator>`） | SP 若非 C++20，改用 `<ranges>` + `std::vector` |

### 5.2 数据模型兼容

- SP 主循环 `src/standard.cpp` L57-80：`camera.read → imu_at → solver.set_R_gimbal2world → detector.detect → tracker.track → aimer.aim → cboard.send`。
- 新增 `OutpostTarget` 不破坏主循环：`Tracker::track` 内部根据 armor name 决定构造哪个 Target，对外仍返回 `std::list<Target>`（需 `OutpostTarget` 与 `Target` 共享接口或基类）。
- **关键决策点**：是否引入基类 `AbstractTarget` 让 `Target`/`OutpostTarget` 共同继承？
  - 引入：Aimer 可无差别处理，但需改 `std::list<Target>` → `std::list<std::unique_ptr<AbstractTarget>>`，工程量大。
  - 不引入：让 `OutpostTarget` 在内部完成所有逻辑后，转换为 `Target` 兼容形式返回（11 维 state 中只填前哨站相关字段，其余置 0）。**推荐此方案**，零侵入。

### 5.3 与地面单位的功能边界

- **Tracker 层**：`Tracker::set_target` 根据 `armor.name == ArmorName::outpost` 走 `OutpostTarget` 分支，其他 name 走原 `Target`。互不干扰。
- **Aimer 层**：`choose_aim_point` 根据 `target.name` 走不同参数（coming/leaving 已分前哨站 70°/30° vs 普通 55°/20°），无需新分支。
- **Shooter 层**：开火容差分档已存在（近 3° / 远 2°），无需改。
- **CAN 发送层**：`io/cboard.cpp` 与目标类型无关，无需改。
- **测试边界**：地面单位回归测试用普通装甲板数据，前哨站测试用 outpost 数据，互不交叉。

---

## 六、测试设计

### 6.1 单元测试

| 测试项 | 测试目的 | 输入 | 期望 |
|---|---|---|---|
| `UT_HeightBuffer_level` | 三块板高度辨识 | 构造 (高, 中, 低) 三次观测序列 | `level()` 依次返回 0/1/2 |
| `UT_HeightBuffer_partial` | 仅观测两块板 | 只填两槽 | `level()` 给出已观测板相对序号，无 NaN 崩溃 |
| `UT_HeightBuffer_clean_oldest` | 三块齐后清理最旧 | 三次观测 + 一次新切板 | 最旧槽变 NaN，新槽写入 |
| `UT_PlateSwitch_single` | 单次切板不锁定方向 | 一帧切板 | `rotation_sign` 仍为 0 |
| `UT_PlateSwitch_confirm` | 连续两次同向锁定 | 两帧同向切板 | `rotation_sign` 锁定为 ±1 |
| `UT_PlateSwitch_opposite` | 反向切板重置候选 | 一右一左切板 | `sign_evidence_count` 重置 |
| `UT_outpost_relative_height` | 四象限高度差表 | (right, upper) 四组合 | 返回 +2/-1/+1/-2 step |
| `UT_transform_to_reference` | 参考板坐标系折算 | rel_see_index=±1, obs=[x,y,z,yaw] | delta_yaw=±2π/3, delta_height 按表 |
| `UT_observation_validation` | 观测位置门控 | 远距离噪声帧 | `is_observation_valid` 返回 false |
| `UT_yaw_interpolation` | 三点二次插值 | err_low/err_opt/err_up 三个采样 | 公式正确，亚度级精化 |

### 6.2 集成测试

| 测试项 | 测试目的 | 输入 | 期望 |
|---|---|---|---|
| `IT_outpost_ekf_converge` | 5 维 EKF 收敛 | rmcs `test_outpost_ekf.cpp` 真值轨迹（中心 (2,0,2), ω=0.8π） + σ=0.03 噪声 | 中心误差 <0.05m, ω 误差 <0.1 rad/s, 角度误差 <0.05 rad |
| `IT_6pt_pnp_distance` | 6 点 PnP 测距 | 人造 5m/8m/10m 远距离板姿态 + 邻侧灯条端点 | 6 点测距误差 < 4 点测距误差的 50% |
| `IT_outpost_switch_plate` | 切板检测+方向锁定 | rmcs test 真值序列中三块板循环切换 | 切板判定准确率 >95%，方向锁定后无反向误判 |
| `IT_outpost_abs_ref_index` | 三块板齐后序号纠正 | 观测序列覆盖三块板 | `abs_ref_index` 与真值一致 |

### 6.3 场景测试

| 测试项 | 场景 | 验证点 |
|---|---|---|
| `ST_outpost_full_chain` | 录像回放真实前哨站视频 | end-to-end：tracker 不发散，aimer 跟踪稳定，shooter 在 coming_angle 范围内开火 |
| `ST_outpost_temp_lost` | 前哨站短暂遮挡后重现 | temp_lost_count=75 帧内恢复跟踪，EKF 状态无突变 |
| `ST_ground_unit_regression` | 普通车录像回归测试 | 地面单位功能零回归：跟踪 / 瞄准 / 开火 完全等同改造前 |
| `ST_outpost_distance_comparison` | 同一前哨站视频，开关 6 点 PnP | 远距离测距精度提升可见，近距离无副作用 |
| `ST_boundary_no_cross` | 视频中前哨站与普通车交替出现 | Tracker 正确切换 target，无类型混淆 |

### 6.4 测试基础设施

- 参考 rmcs `tool/cxx/test_outpost_ekf.cpp`：构造人造真值轨迹 + 高斯噪声 + 离群点，验证 EKF 收敛精度。SP 版本移除 ROS marker publisher，改为打印统计到 stdout / 写入 csv 后用 Python 绘图。
- 集成测试用 SP 现有录像回放框架（`src/io/recorder/player.cpp` 的 SP 对应物）。

---

## 七、风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| SP detector 不输出邻侧灯条 2D 端点 | 6 点 PnP 无法直接套用 | 降级方案：用 4 装甲板角点 + 由阶段 2.1 几何推算的虚拟邻侧灯条 3D 点构造对应关系，仍能起到几何约束作用 |
| rmcs 与 SP 的 yaw 符号约定不一致 | 折算方向反了 | 在阶段 1.3 移植时先用单板静止数据校验符号，对照 SP `armor.ypr_in_world[0]` 的物理含义（armor→center 还是 center→armor） |
| SP `tools::ExtendedKalmanFilter` 动态维度切 5 维后与 11 维路径冲突 | 普通车回归失败 | 策略 A 隔离方案：新建 `OutpostTarget` 类用独立 EKF 实例，不动现有 `Target` |
| 阶段 3 改动大、回归风险高 | 普通车功能受影响 | 阶段 1-2 先合入（独立工具层 + 6 点 PnP），阶段 3-5 分 PR 合入，每个 PR 跑回归 |
| 切板方向阈值在不同场地光照下需调参 | 误判切板 | YAML 暴露 `plate_switch_yaw_min` / `sign_confirm_count`，初始值按 rmcs 默认 50° / 2 |

---

## 八、实施次序建议（按 PR 粒度）

1. **PR-1**：阶段 1 全部 + 单元测试（独立工具层，零侵入）
2. **PR-2**：阶段 2（6 点 PnP 距离优化器 + 邻侧板几何）
3. **PR-3**：阶段 3（`OutpostTarget` 类 + 5 维 EKF + 板序号 / 切板 / 折算 接入）+ 集成测试
4. **PR-4**：阶段 4（Tracker 接入 + 修 4 个已知缺陷 + 图像中心硬编码）
5. **PR-5**：阶段 5（Aimer 改造：动态窗口 + 滞回 + 预瞄）
6. **PR-6**：阶段 6（场景测试 + 录像回放验证）

每个 PR 必须跑通：
- 单元测试全绿
- 地面单位录像回归测试无差异
- 前哨站录像场景测试通过对应阶段的功能点

---

## 九、关键源码引用（便于实施时定位）

### rmcs 侧
- `D:\RM\rmcs_auto_aim_v2\src\module\tracker\model\outpost.cpp` — OutpostModel 主体（HeightBuffer、切板、折算、EKF）
- `D:\RM\rmcs_auto_aim_v2\src\module\tracker\model\outpost.hpp` — 接口与 Config
- `D:\RM\rmcs_auto_aim_v2\src\utility\math\outpost.cpp` — `outpost_relative_height` / `OutpostSolution` / `NeighborBarSolution`
- `D:\RM\rmcs_auto_aim_v2\src\utility\math\solve_pnp\outpost_distance_optimizer.cpp` — 6 点 PnP 距离优化器
- `D:\RM\rmcs_auto_aim_v2\src\utility\math\solve_pnp\pnp_solution.cpp` — RobustPnpSolution（含 yaw 优化 + 三点插值）
- `D:\RM\rmcs_auto_aim_v2\src\utility\math\kalman_filter\ekf.hpp` — EKF 模板（Joseph form）
- `D:\RM\rmcs_auto_aim_v2\src\utility\robot\constant.hpp` — 几何常量
- `D:\RM\rmcs_auto_aim_v2\src\utility\math\angle.hpp` — `normalize_angle` / `deg2rad`
- `D:\RM\rmcs_auto_aim_v2\src\kernel\fire_control.cpp` — `evaluate_armor` / `get_attack_window` / `boundary_point`
- `D:\RM\rmcs_auto_aim_v2\src\module\tracker\trackable.hpp` — Trackable 抽象（仅参考，不移植）
- `D:\RM\rmcs_auto_aim_v2\tool\cxx\test_outpost_ekf.cpp` — EKF 测试参考

### SP 侧
- `d:\RM\sp_vision_25-main\tasks\auto_aim\target.cpp` / `target.hpp` — 现有 11 维 Target
- `d:\RM\sp_vision_25-main\tasks\auto_aim\tracker.cpp` / `tracker.hpp` — 现有 Tracker（L280 多板循环缺陷）
- `d:\RM\sp_vision_25-main\tasks\auto_aim\aimer.cpp` / `aimer.hpp` — 现有 Aimer（L150 死跟 id=0 缺陷）
- `d:\RM\sp_vision_25-main\tasks\auto_aim\solver.cpp` / `solver.hpp` — 现有 Solver（无 6 点 PnP）
- `d:\RM\sp_vision_25-main\tasks\auto_aim\armor.hpp` — Armor 数据结构
- `d:\RM\sp_vision_25-main\tools\extended_kalman_filter.hpp` — SP EKF 类（动态维度）
- `d:\RM\sp_vision_25-main\src\standard.cpp` — 主循环
- `d:\RM\sp_vision_25-main\io\cboard.cpp` — CAN 发送
- `d:\RM\sp_vision_25-main\configs\standard3.yaml` — 配置

---

## 十、待用户确认事项

在进入编码阶段前，请确认以下决策：

1. **策略 A vs 策略 B**（阶段 3）：新建独立 `OutpostTarget` 类（推荐，隔离风险） vs 在现有 `Target` 类内加分支（改动集中但臃肿）。
2. **邻侧灯条 2D 端点是否可从 SP detector 拿到**：决定 6 点 PnP 走原版还是降级方案（用几何推算的虚拟灯条 3D 点）。
3. **`kOutpostRadius` 取值**：SP 旧值 0.2765 vs rmcs 值 0.275，差异 1.5mm，建议沿用 SP 旧值以免引入未标定误差。
4. **是否引入 `AbstractTarget` 基类**：影响 Aimer / Tracker 接口签名（推荐不引入，`OutpostTarget` 内部转 `Target` 兼容形式返回）。
5. **PR 拆分粒度**：是否按本文档建议的 6 个 PR 拆分，或合并为更少 / 更多 PR。
6. **测试基础设施**：是否引入 GoogleTest 框架，还是沿用 SP 现有测试风格（若 SP 现有测试框架已存在，请告知路径）。

确认后即可进入编码实施阶段。
