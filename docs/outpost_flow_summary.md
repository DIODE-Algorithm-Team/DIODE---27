# 前哨站（Outpost）自瞄流程总结

> 本文是**已落地实现**的流程说明（含代码位置、使用方法、参数、测试与排查要点）。
> 设计/迁移方案（rmcs → sp_vision 的分阶段计划、可复用模块分析、待确认事项）见
> [outpost_migration.md](outpost_migration.md)；两者冲突时以本文与实际代码为准。

前哨站链路的核心思路：**前哨站几何完全固定**（三块板 120° 均布、高差 0.102 m、
半径 0.2765 m、板 pitch −15°），因此放弃 SP 原有的 11 维「整车 EKF + 球坐标观测」，
改用 rmcs 的 **5 维状态 + 4 维 3D 观测**，并把邻侧板几何推算出的灯条端点作为
**6 点 PnP** 的额外约束来压制远距离测距退化；对外仍以 11 维 `Target` 快照返回，
使 `Aimer` / `Shooter` / 状态机零改动复用。

---

## 1. 端到端数据流

```
io::Camera ──img,t──► 识别器 (YOLO/Detector)           tasks/auto_aim/yolo*.cpp
                          │  Armor{name=outpost, type=small(135mm)}
                          ▼
                      Solver::solve()                 4 点 IPPE + optimize_yaw(三点插值)
                          │  armor.xyz_in_world / ypr_in_world / ypd_in_world
                          ▼
        ┌────────────► Tracker::set_target()  ── armor.name==outpost ──┐
        │                                                              ▼
        │                                       OutpostTarget  5 维 EKF (3D 观测)
        │                                         ├ is_observation_valid  xy 门控
        │                                         ├ PlateSwitchDetector  切板方向×2 锁定
        │                                         ├ HeightBuffer         板序号在线辨识
        │                                         ├ transform_to_reference 折算回参考板
        │                                         └ EKF update [cx,cy,cz,armor_yaw]
        │                                                              │
        │             Tracker::update_target()                          │
        │               ├ 逐个 armor solve 后与三板预测位姿匹配取最优板  │
        │               ├ Solver::solve_outpost_6pt()  ← 6 点 PnP 距离优化
        │               └ OutpostTarget::update(best_armor)             │
        │                                                              ▼
        └──────────── Tracker::refresh_outpost_snapshot()  → 11 维 Target 快照
                          │  Tracker::track() 对外返回 std::list<Target>
                          ▼
                      Aimer::aim() → choose_aim_point() → evaluate_outpost()
                          动态攻击窗口 / 施密特滞回锁定 / 边界点预瞄
                          │  io::Command{control, shoot, yaw, pitch}
                          ▼
                      Shooter::shoot()  (读 aimer.debug_aim_point.valid)
                          ▼
                      io::CBoard::send()
```

关键点：**没有独立的「前哨站模式」开关**——只要识别结果的 `armor.name == ArmorName::outpost`，
`Tracker` 就自动切到 5 维链路；`io::Mode::outpost` 只是无人机 C 板的一个档位，
其处理与 `auto_aim` 档完全相同（见 §5.4）。

---

## 2. 代码位置总览

### 2.1 前哨站专用文件（新增）

| 文件 | 关键位置 | 职责 |
|---|---|---|
| [outpost_const.hpp](../tasks/auto_aim/outpost_const.hpp) | 全文（`namespace auto_aim::outpost_const`） | 几何/逻辑常量：`kRadius` L14、`kArmorThickness` L18、`kHeightStep` L22、`kAngleStep` L25、`kPredictedPitch` L29、`kPlateSwitchYawMin` L34、`kSignConfirmCount` L38、`kOffsetTable` L49 |
| [outpost_helper.hpp](../tasks/auto_aim/outpost_helper.hpp) | `HeightBuffer` L25-117 | 板序号在线辨识（三槽位、`kFactor=0.1` 指数加权、NaN 视为未观测） |
| 同上 | `PlateSwitchDetector` L125-152 | 切板方向连续 `kSignConfirmCount`(=2) 次同向才锁定，抗单帧噪声 |
| 同上 | `outpost_relative_height()` L165-172 | 四象限邻板高度差表（+2/−1/+1/−2 step） |
| 同上 | `transform_to_reference()` L183-193 | 当前板观测折算回参考板（高度 −Δh、yaw −Δyaw） |
| 同上 | `is_observation_valid()` L199-207 | 位置门控：`xy_error` 超限（>5 m 用 0.20，否则 0.10）丢弃该帧 |
| [outpost_geometry.hpp](../tasks/auto_aim/outpost_geometry.hpp) | `OutpostSolution` L26-121 | 由当前板位姿推算邻侧板位姿（`solve` L44-86、`get_transform` L90-109、`get_level` L113-120） |
| 同上 | `NeighborBarSolution` L128-204 | 推算邻侧上/下灯条 3D 端点（`near`/`away` 各 4 组），供 6 点 PnP 使用 |
| [outpost_target.hpp](../tasks/auto_aim/outpost_target.hpp) | `class OutpostTarget` L37-84 | 前哨站专用目标（5 维 EKF），与 11 维 `Target` 并列、互不干扰 |
| [outpost_target.cpp](../tasks/auto_aim/outpost_target.cpp) | `make_observation` L30-39 | `Armor` → 观测 `[center.x, center.y, center.z, armor_yaw]` |
| 同上 | 噪声常量 L12-20 | 过程噪声 Q、观测噪声 R（对齐 rmcs `OutpostModel::Config` 默认值） |
| 同上 | 构造 L43-84 | 首块观测板即参考板，`P0 = diag{1,1,1,64,0.4}`、`x=[center, ω=0, ref_yaw]` |
| 同上 | `predict` L86-117 | `F(4,3)=dt`，`angle = limit_rad(angle + ω·dt)` |
| 同上 | `update` L119-199 | 门控 → 切板检测 → 三等板齐纠正 `abs_ref_index` → 参考板折算 → EKF 更新 → 维护 `last_observation_` |
| 同上 | `armor_xyza_list` L205-229 | 三块板 `[x,y,z,a]`（高→中→低），`a` = armor→center 方向 yaw，可直接与 `Armor.ypr_in_world[0]` 比较 |
| 同上 | `diverged` L231-245 / `convergened` L247-254 | 发散判定（超时 500 ms、协方差>150、\|中心\|>15 m、\|ω\|>10π、NaN）／收敛判定（≥5 帧且 cov(0,0),(1,1),(4,4)<0.01） |

### 2.2 主链路接入点

| 文件 | 关键位置 | 说明 |
|---|---|---|
| [tracker.hpp](../tasks/auto_aim/tracker.hpp) | L41、L45-49、L61-62 | `outpost_max_temp_lost_count_`、`is_outpost_`、`std::optional<OutpostTarget>`、`refresh_outpost_snapshot()` 声明 |
| [tracker.cpp](../tasks/auto_aim/tracker.cpp) | L21 | 读 YAML `outpost_max_temp_lost_count`（**无 `IsDefined` 保护，必需键**） |
| 同上 | L276-282（`set_target`） | `armor.name == outpost` → `is_outpost_=true`、构造 `OutpostTarget`、刷新快照；其他兵种仍走 11 维 `Target` |
| 同上 | L299-345（`update_target`） | 前哨站分支：`predict` → 与三板预测位姿按「板朝向差 + 方位角差」取最优板 → `solve_outpost_6pt` → `OutpostTarget::update` → 刷新快照（**每帧只 update 一块板**，修掉原先多板循环 update） |
| 同上 | L375-394（`refresh_outpost_snapshot`） | 5 维 → 11 维快照映射（换算见 §3.2），并保留 `priority` |
| 同上 | L84-90、L183-189、L93-100 | 发散检测 / NIS 失败比例检查改用 `outpost_target_->diverged()`、`outpost_target_->ekf()` |
| 同上 | L241-245 | `state_=="temp_lost"` 且目标为前哨站时用更大的丢帧容忍（`outpost_max_temp_lost_count_`） |
| [solver.hpp](../tasks/auto_aim/solver.hpp) | L36-38 | `solve_outpost_6pt` 声明 |
| [solver.cpp](../tasks/auto_aim/solver.cpp) | L94-199 | 6 点 PnP：4 点 IPPE 热启动 → `NeighborBarSolution` 推算端点 → 世界系端点 `world2pixel` 投影（哨兵模式无 2D 端点时）→ 6 点 `SOLVEPNP_ITERATIVE` → 写回世界系 → 重新 `optimize_yaw` |
| 同上 | L207 | `reproject_armor` 中前哨站用 pitch = −15°（其余 +15°） |
| 同上 | L327-347 | `optimize_yaw` 三点二次插值精化（阶段 2 的 yaw 优化升级，已落地） |
| [target.hpp](../tasks/auto_aim/target.hpp) | L32-39 | 11 维快照构造 `Target(x_state, name, type, armor_num, converged)` |
| [target.cpp](../tasks/auto_aim/target.cpp) | L69-95 | 快照构造：`jumped=true`、`P=0`（只做前向外推，不做 update） |
| 同上 | L126-131 | 前哨站过程噪声 `v1=10, v2=0.1`（普通车 100/400） |
| 同上 | L160-162 | 前哨站转速特判：收敛且 \|ω\|>2 时钳到 ±2.51 rad/s |
| 同上 | L281-293 | `convergened()` 前哨站门控 `update_count_>10` |
| 同上 | L295-315 | `h_armor_xyz` 前哨站三板高度台阶 `{0, −step, −2·step}`（快照 `x[4]` 为最高板高度） |
| [aimer.hpp](../tasks/auto_aim/aimer.hpp) | L15-20、L46-70 | `AimPoint{valid, shootable, xyza}`、前哨站开火控制参数与接口 |
| [aimer.cpp](../tasks/auto_aim/aimer.cpp) | L32-46 | 读取 PR-5 开火控制参数（全部带缺省值） |
| 同上 | L58-65 | 入口退化保护：`\|ω\| > degraded_angle_speed_` 直接不输出云台控制 |
| 同上 | L185 | `choose_aim_point` 对 `outpost` 分流到 `evaluate_outpost` |
| 同上 | L241-306 | 动态攻击窗口 + 施密特滞回锁定 + 窗口内无板时边界预瞄 |
| 同上 | L308-348 | `get_attack_window`（rmcs `FireController::get_attack_window`，按板数泛化周期角） |
| 同上 | L350-378 | `outpost_boundary_point`（来板一侧窗口边界预瞄点） |
| 同上 | L152-154 | `shootable==false` → `debug_aim_point.valid=false`（预瞄只跟不打） |
| [shooter.cpp](../tasks/auto_aim/shooter.cpp) | L19-41 | 开火判据之一即 `aimer.debug_aim_point.valid`（另两个是 command.yaw 突变与云台到位容差） |
| [decider.cpp](../tasks/omniperception/decider.cpp) | L143-144 | 哨兵 `armor_filter` **过滤掉前哨站**（哨兵不打前哨站） |

### 2.3 外围

| 文件 | 位置 | 说明 |
|---|---|---|
| [io/cboard.hpp](../io/cboard.hpp) | L18-26 | `io::Mode::outpost`（值 4）、`MODES` 第 5 项 |
| [src/uav.cpp](../src/uav.cpp) | L78 | `mode == auto_aim \|\| mode == outpost` 走同一套自瞄流程（无人机有独立前哨站档） |
| [src/sentry.cpp](../src/sentry.cpp) | L79、L85-96 | 哨兵：`decider.armor_filter` 过滤前哨站后才 `tracker.track` |
| [src/standard.cpp](../src/standard.cpp) | L75-77 | 步兵：无过滤，识别到前哨站即自动进入 5 维链路 |
| [yolos/yolov8.cpp](../tasks/auto_aim/yolos/yolov8.cpp) | L198-209、L218-222 | `check_type`/`get_type`：前哨站只能是小装甲板 |
| [yolos/yolov5.cpp](../tasks/auto_aim/yolos/yolov5.cpp) L211、[yolos/yolo11.cpp](../tasks/auto_aim/yolos/yolo11.cpp) L194 | — | 同上（大板假设下排除 outpost） |
| [assets/yolo11.xml](../assets/yolo11.xml) | L23426 | 类别表含 `Boutpost / Routpost / Eoutpost`（与 `armor_properties` 顺序对应） |
| [CMakeLists.txt](../CMakeLists.txt) | L105-131 | 4 个前哨站测试目标的定义 |
| [docs/outpost_migration.md](outpost_migration.md) | 全文 | 设计/迁移方案（历史文档） |

---

## 3. 数据约定

### 3.1 `OutpostTarget` 5 维状态 / 4 维观测（[outpost_target.hpp](../tasks/auto_aim/outpost_target.hpp) L20-30）

| 量 | 含义 |
|---|---|
| `x[0..2]` | 旋转中心（参考板高度处的虚拟中心）世界系 x/y/z |
| `x[3]` | 转速 ω (rad/s) |
| `x[4]` | 参考板 center→armor 方向 yaw (rad) |
| 观测 `z[0..2]` | 板中心 `center = armor.xyz_in_world + radius · armor2center`（`armor2center` 已绕板 Y 轴反转 −pitch，水平指向旋转中心） |
| 观测 `z[3]` | `armor_yaw = atan2(−armor2center.y, −armor2center.x)`（center→armor，与 SP `ypr[0]` 反向，仅在类内部使用） |

EKF 观测矩阵 `H` 取 `x,y,z,yaw` 四行（[outpost_target.cpp](../tasks/auto_aim/outpost_target.cpp) L171-176），
即 **ω 不可观、由 yaw 的时序变化间接估计**；R = `diag{5e-3, 5e-3, 1e-2, 1e-2}`、
Q = `diag{1e-6, 1e-6, 1e-5, 1e-4, 1e-4}`（同文件 L12-20、L106）。

### 3.2 5 维 → 11 维快照映射（[tracker.cpp](../tasks/auto_aim/tracker.cpp) L375-394）

```
11 维 Target: [cx, vx, cy, vy, cz, vz, a, ω, r, l, h]
  cx,cy,cz = s[0], s[1], s[2] − ref_h_off     // 扣除参考板高度偏移 → x[4] 表示最高板高度
  a        = limit_rad(s[4] + π − ref_yaw_off) // center→armor 转 armor→center，再对齐绝对板序
  ω        = s[3]
  r        = outpost_const::kRadius
  v*, l, h = 0                                  // 前哨站不移动、无长短轴/高度差估计
```

`ref_yaw_off/ref_h_off` 取自 `kOffsetTable[abs_ref_index()]`；三板高度台阶由
[target.cpp](../tasks/auto_aim/target.cpp) L295-315 的 `h_armor_xyz` 给出，保证瞄准中/低板时 pitch 正确。

### 3.3 几何常量（[outpost_const.hpp](../tasks/auto_aim/outpost_const.hpp)）

| 常量 | 值 | 备注 |
|---|---|---|
| `kRadius` | 0.2765 m | 沿用 SP 旧值（rmcs 为 0.275，差 1.5 mm，见迁移文档 §10） |
| `kArmorThickness` | 0.008 m | 传给 `OutpostSolution`/`solve_outpost_6pt` |
| `kHeightStep` | 0.102 m | 三板高差 |
| `kAngleStep` | 2π/3 | 三板 120° |
| `kPredictedPitch` | −15° | 前哨站板 pitch（普通车 +15°，见 solver.cpp L207） |
| `kPlateSwitchYawMin` | 50° | 切板判定阈值 |
| `kSignConfirmCount` | 2 | 方向锁定所需连续同向切板次数 |
| `kAngularSpeed` | 0.8π rad/s | 仅测试真值轨迹用 |
| `kOffsetTable` | `{0,0}, {120°,−1}, {240°,−2}` | 高→中→低 |

---

## 4. 流程详解

### 4.1 跟踪目标建立（`Tracker::set_target`，[tracker.cpp](../tasks/auto_aim/tracker.cpp) L253-295）

1. 取排序后的首块装甲板（先按图像中心距、再按 `priority` 升序），`solver_.solve(armor)`；
2. 复位 `is_outpost_/outpost_target_`；若 `armor.name == ArmorName::outpost`：
   `is_outpost_=true`、记录 `outpost_priority_`、`outpost_target_.emplace(armor, t)`、刷新快照；
3. 平衡（大板 3/4/5）、基地（半径 0.3205、三板）、其余（半径 0.2、四板）仍走原 11 维 `Target`。

### 4.2 每帧更新（`Tracker::update_target`，同文件 L297-373）

1. `outpost_target_->predict(t)`；
2. 遍历画面中所有 `outpost` 小装甲板，各自 `solve` 后与 `armor_xyza_list()` 的三块板预测位姿比较，
   取 `|Δyaw| + |Δ方位角|` 最小者为 `best_armor`（每帧只喂一块板）；
3. `sign = rotation_sign()`，`is_right = (sign < 0)`，`is_upper = true`，
   调用 `solver_.solve_outpost_6pt(*best_armor, is_right, true, {-1,-1}, {-1,-1}, kArmorThickness)`；
   2D 端点传占位值 `(-1,-1)` 时，`solver` 内部用 `world2pixel` 把几何推算出的世界系端点投回图像
   （纯几何闭环，无额外观测，见 §6.3）；
4. `outpost_target_->update(*best_armor)`；无论本帧是否匹配到板，最后都 `refresh_outpost_snapshot()`，
   使 `temp_lost` 期间 `Aimer` 仍拿到 predict 后的最新外推位姿；
5. 返回 `found` 供状态机使用（`min_detect_count_` 帧后进入 `tracking`）。

### 4.3 观测更新内部（[outpost_target.cpp](../tasks/auto_aim/outpost_target.cpp) L119-199）

1. **门控**：`is_observation_valid(obs, last_observation_)` 不通过则整帧丢弃（不更新 EKF）；
2. **切板检测**：`|limit_rad(obs_yaw − last_yaw)| > 50°` 视为真实切板 → `sign = ±1`
   → `PlateSwitchDetector::update(sign)`（未锁定时用单帧 sign 兜底）
   → `HeightBuffer::go_next(go_sign)` + `update(z)` → `rel_see_index_ ±1` 折叠到 {−1,0,+1}；
   三块板齐后 `abs_ref_index_ = (level() − rel_see_index_ + 3) % 3` 并 `clean_oldest()`；
3. **折算**：`Δyaw = ±120°`（按 `rel_see_index_` 符号）、`Δh = outpost_relative_height(in_right, in_upper)`，
   其中 `in_upper = obs_z > ekf_.x[2]`；折算后做 `H/R` 线性更新；
4. **`last_observation_` 维护**：用后验状态反补偿回当前板坐标系，使下一帧切板检测仍以当前板为基准。

### 4.4 决策与开火（[aimer.cpp](../tasks/auto_aim/aimer.cpp)）

1. `Aimer::aim` 先做退化保护（L58-65）：快照 `ekf_x()[7]`（原始 ω）> `degraded_angle_speed_`(默认 12 rad/s)
   → 返回 `{false,false,0,0}` 并清空锁定；前哨站 0.8π≈2.51 不会触发；
2. 时间补偿 + 弹道迭代预测后 `choose_aim_point` → `evaluate_outpost`（L241-306）：
   - 动态窗口半幅 `half = min(attack_window_/2, get_attack_window(...))`，
     `get_attack_window` 用锯齿波模型按云台 `max_yaw_vel/max_yaw_acc` 反解目标本体角度半幅
     （ω<0.1 不限、ω>12 退化 NaN）；
   - 施密特滞回：`enter = half·(1−hyst)`、`exit = half·(1+hyst)`；已锁定板在 `exit` 内保持，
     否则在 `enter` 内选 `|delta|` 最小的板并锁定（`outpost_lock_id_` 跨帧保持）；
   - 窗口内无板：清空锁定，返回**来板一侧**（ω<0 → `+half`，ω>0 → `−half`）的
     `outpost_boundary_point` 预瞄点，`shootable = attack_preaim_`（默认 false：只跟踪不开火）；
3. `Aimer::aim` 末尾把 `shootable==false` 转成 `debug_aim_point.valid=false`（L152-154），
   `Shooter::shoot` 因此不开火（[shooter.cpp](../tasks/auto_aim/shooter.cpp) L34）。

### 4.5 异常与退化

| 情形 | 行为 | 位置 |
|---|---|---|
| 观测 xy 跳变超阈值 | 丢弃该帧，EKF 不更新 | outpost_target.cpp L125 |
| 协方差/位置/ω/NaN 异常、>500 ms 未更新 | `diverged()` → `Tracker` 置 `lost`、返回空 | outpost_target.cpp L231-245、tracker.cpp L84-90 |
| NIS 失败比例 ≥ 窗口 40% | 置 `lost` | tracker.cpp L93-100 |
| 丢失目标 | 前哨站用 `outpost_max_temp_lost_count`（75/150）而非 `max_temp_lost_count` | tracker.cpp L241-245 |
| 大 dt（相机离线） | `dt > 0.1 s` → 置 `lost` | tracker.cpp L43-46 |
| ω 过大 | `Target::predict` 快照钳到 ±2.51 rad/s；`Aimer` 入口再挡 >12 rad/s | target.cpp L160-162、aimer.cpp L58-65 |

---

## 5. 使用方法

### 5.1 配置文件（必需键）

`Tracker` 构造时**无条件**读取 `outpost_max_temp_lost_count`（[tracker.cpp](../tasks/auto_aim/tracker.cpp) L21），
缺失会直接抛 YAML 异常。所有兵种的运行配置都必须有该键：

| 配置文件 | 值 | 备注 |
|---|---|---|
| [configs/sentry.yaml](../configs/sentry.yaml) L63 | 75 | 同时含 PR-5 开火控制参数 L76-82 |
| [configs/standard3.yaml](../configs/standard3.yaml) L37、[standard4.yaml](../configs/standard4.yaml) L38 | 75 | 步兵 |
| [configs/demo.yaml](../configs/demo.yaml) L38、[ascento.yaml](../configs/ascento.yaml) L38、[mvs.yaml](../configs/mvs.yaml) L66 | 75 | 调试/其他 |
| [configs/uav.yaml](../configs/uav.yaml) L66 | 150 | 无人机（前哨站丢失容忍更长） |
| [configs/example.yaml](../configs/example.yaml) L57-58 | **缺失** | 新配置模板：**需手动补 `outpost_max_temp_lost_count`** |

### 5.2 可选调参键（`Aimer`，全部有缺省值）

[aimer.cpp](../tasks/auto_aim/aimer.cpp) L32-46；目前仅 [configs/sentry.yaml](../configs/sentry.yaml) L76-82 显式列出。

| 键 | 缺省 | 作用 |
|---|---|---|
| `attack_window` | 40° | 攻击窗口全幅（与动态窗口取 min） |
| `window_hysteresis` | 0.2 | 施密特滞回系数（enter=half·0.8 / exit=half·1.2） |
| `degraded_angle_speed` | 12 rad/s | 超过则不输出云台控制；前哨站 0.8π 不触发 |
| `window_redundancy` | 0.8 | 云台速度/加速度裕度 |
| `max_yaw_vel` | 10 rad/s | 云台最大 yaw 角速度（动态窗口反解用） |
| `max_yaw_acc` | 200 rad/s² | 云台最大 yaw 角加速度 |
| `attack_preaim` | false | 边界预瞄点是否允许开火（false=只跟踪不开火） |

常量类参数（半径、高差、pitch、切板阈值、方向确认次数）目前**编译期固定**在
[outpost_const.hpp](../tasks/auto_aim/outpost_const.hpp)，改需重新编译（迁移文档 L383 曾建议暴露到 YAML，尚未实现）。

### 5.3 编译

项目目标平台为 Ubuntu（见 [readme.md](../readme.md) §3.2）：

```bash
cmake -B build
make -C build -j$(nproc)
```

注意 [CMakeLists.txt](../CMakeLists.txt) L15 硬编码了 `OpenVINO_DIR=/home/gzy/openvino_env/...`，
换机器需先改该路径；且 L16 是无条件的 `find_package(OpenVINO REQUIRED)`，
即使只想编译前哨站单测（这些目标本身不链接 OpenVINO，见 §5.6），配置阶段也需要 OpenVINO 可用。

### 5.4 运行

前哨站链路随识别结果自动生效，无需开关：

```bash
# 步兵 / 无人机：识别到 outpost 即走 5 维链路
./build/standard configs/standard3.yaml
./build/uav      configs/uav.yaml        # C 板 mode=outpost(4) 与 auto_aim(1) 等价
# 哨兵：Decider::armor_filter 会过滤掉前哨站（规则上不打），故哨兵不会进入该链路
./build/sentry   configs/sentry.yaml
```

`io::Mode::outpost` 是下位机下发的档位编号 4（[io/cboard.hpp](../io/cboard.hpp) L18-26），
只有 [src/uav.cpp](../src/uav.cpp) L78 显式把 `outpost` 与 `auto_aim` 并列处理。

### 5.5 离线回放与调试

```bash
./build/auto_aim_test -c configs/demo.yaml -s 0 -e 0 assets/demo/demo
```

- `auto_aim_test` 用 `YOLO + Solver + Tracker + Aimer` 跑录像，前哨站录像同样走新链路；
- `src/sentry_debug.cpp`、`src/uav_debug.cpp`、`src/mt_auto_aim_debug.cpp` 都会打印/绘制
  `aimer.debug_aim_point`（`valid`=是否开火、`shootable`=是否预瞄、`xyza`=瞄准点），
  是观察攻击窗口/滞回/预瞄行为最直接的入口；
- 日志关键字：`[Tracker] Target diverged!`（发散）、`[Target] Bad Converge Found!`（NIS 收敛差）、
  `[Aimer] Outpost attack window degraded: omega=...`（ω 过大退化）、`[Tracker] Large dt`（相机离线）。

### 5.6 前哨站测试

已在 [CMakeLists.txt](../CMakeLists.txt) L105-131 注册 4 个测试目标。
一键脚本：[tests/run_outpost_tests.sh](../tests/run_outpost_tests.sh)（默认走 CMake 目标；
`--standalone` 用 g++ 直编、不需要 OpenVINO 与相机 SDK；`--clean`/`--regression`/`--config` 可选）。

```bash
# 推荐：一键编译 + 运行 + 汇总（退出码 0=全通过）
bash tests/run_outpost_tests.sh

# 等价的手工命令
cmake --build build --target outpost_helper_test outpost_geometry_test outpost_target_test outpost_aimer_test -j
./build/outpost_helper_test      # PR-1 工具层单测
./build/outpost_geometry_test    # PR-2 邻侧板几何推算
./build/outpost_target_test      # PR-3 5 维 EKF 集成（真值轨迹 + 高斯噪声）
./build/outpost_aimer_test       # PR-6 快照 → Aimer 链路（动态窗口/滞回/预瞄）
```

| 测试 | 覆盖内容 | 头注释 |
|---|---|---|
| [outpost_helper_test.cpp](../tests/outpost_helper_test.cpp) | HeightBuffer 单/双/三板与清理、方向二次确认、四象限高度差、参考板折算、门控阈值 | L1-10 |
| [outpost_geometry_test.cpp](../tests/outpost_geometry_test.cpp) | 同源等价、旋转中心水平偏移 = r+thickness、UPPER→MIDDLE 解析值、三板等半径、near/away 端点、灯条高 56 mm、`get_level` 四象限、切板循环闭合 | L1-13 |
| [outpost_target_test.cpp](../tests/outpost_target_test.cpp) | 收敛精度（中心<0.05 m、ω<0.1 rad/s、yaw<0.05 rad）、切板方向锁定与 `rel_see_index` 折叠、`abs_ref_index` 纠正、三板半径恒定、噪声帧门控 | L1-14 |
| [outpost_aimer_test.cpp](../tests/outpost_aimer_test.cpp) | 快照三板 xyz（含高度台阶）与 `armor_xyza_list` 一致、首见非最高板时绝对高度仍为高/中/低、窗口内开火 vs 窗口外预瞄、开火点高度只在 {2.0,1.898,1.796}、预瞄点在来板一侧轨道上、ω 退化不输出 | L1-13 |

> **实测结果**（源码未改动，本机 Windows + MSVC 14.44 用等价目标编译，构建工程见
> `out/win_outpost_tests/`）：`outpost_helper_test` PASS 41 / FAIL 0、
> `outpost_geometry_test` 22 passed / 0 failed、`outpost_target_test` 28 passed / 0 failed、
> `outpost_aimer_test` 7327 passed / 0 failed（三场景 fire_ratio≈0.33，`no_control=0`，
> `snapshot_mismatch=0`）。仓库 `build/` 下的同名文件是 Linux ELF，Windows 不能直接运行，
> 故本机用等价工程复编；Linux 上位机请直接用上面的 `tests/run_outpost_tests.sh`。
>
> Windows 一键运行（脚本保存为 UTF-8 with BOM，PS 5.1 必需）：
> `powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_outpost_tests.ps1`。
> 想在 VS 里断点调试这 4 个测试：用 VS「打开文件夹」打开 `out/win_outpost_tests`
> （内含 `CMakePresets.json`，配置预设 `win-msvc`），而不是打开仓库根目录——
> 仓库根 CMakeLists 依赖 OpenVINO 与 Linux 专有的 `io` 目标，Windows 上无法 configure。
> 注：`outpost_aimer_test` 的通过数在 7325~7327 间小幅浮动（快照内部时间戳取
> `steady_clock::now()`，边界帧在 fire/preaim 之间翻转），失败数恒为 0。

---

## 6. 与上游 / 其他兵种的差异与限制

1. **只支持小装甲板**：识别侧 `check_type/get_type` 强制 outpost 为 small（135 mm 宽、灯条 56 mm）。
2. **哨兵不打前哨站**：[decider.cpp](../tasks/omniperception/decider.cpp) L143-144 直接 remove，
   因此 `sentry.yaml` 里的前哨站参数实际不参与比赛行为（仅保证 `Tracker` 构造不报错）。
3. **6 点 PnP 的额外 2 个点是几何推算的**：`tracker.cpp` 传入 `{-1,-1}` 占位，
   `solver.cpp` L138-149 用 `world2pixel` 把推算出的世界系端点投回图像。
   即当前实现是「纯几何闭环」，并未使用检测器真实检出的邻侧灯条 2D 端点
   （迁移文档 §10 待确认项之一）。
4. **半径沿用 SP 旧值 0.2765**（rmcs 0.275），未重新标定。
5. **常量未暴露 YAML**（见 §5.2 末）。
6. **快照 ω 钳位**：`Target::predict` 对收敛后的前哨站把 |ω|>2 钳到 2.51 rad/s
   （[target.cpp](../tasks/auto_aim/target.cpp) L160-162），而 0.8π≈2.5133，
   故快照外推的转速会比 5 维 EKF 原始值低约 0.3%（量级可忽略，但排查角速度相关现象时需知道）。
7. **实现与迁移文档的差异**：文档 §3 策略 A 提到用 `std::variant`/基类统一管理，
   实际实现是 `std::optional<OutpostTarget> + 11 维快照 Target`（零侵入，
   对 `Aimer`/`Shooter`/状态机透明）；文档 §4.2 的 coming/leaving 70°/30° fallback 未采用，
   前哨站完全走 `evaluate_outpost` 的动态窗口路径。

---

## 7. 相关文档

- 设计/迁移方案：[outpost_migration.md](outpost_migration.md)
- 项目总体说明与编译部署：[readme.md](../readme.md)
- 标定相关：[ChArUco_Calibration_Guide.md](../ChArUco_Calibration_Guide.md)
- 串口迁移记录：[capture_serial_port_migration.md](../capture_serial_port_migration.md)
