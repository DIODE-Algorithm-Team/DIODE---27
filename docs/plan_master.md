# DIODE-27 自瞄升级 · 主计划（Master Plan）

> 单一入口。细节文档见 §7。最近更新：2026-11-02。

---

## 0. 约束与已确认决策

| # | 约束/决策 | 落地 |
|---|---|---|
| 1 | 所有产出必须在 `D:\RM\DIODE-27` 内 | 只写 `docs/ tools/ tasks/ tests/ configs/`，不引用外部副本 |
| 2 | 装甲板物理上已去灯条，但**优先跑通**，识别走原链路 | 不碰灯条/邻侧端点/YOLO-pose；4 点 PnP 与现有模型不动 |
| 3 | 采用 rmcs 调试方案 | 推流 + 浏览器控制台（s 保留）；Feishu shm 不搬 |
| 4 | 项目去 ROS 化 | `io/ros2` 已删；sentry 系列无条件编译 |
| 5 | 导航 ↔ 自瞄**无信息传递** | 不留 UDP 替代，直接删；`Decider` 接口保留 |
| 6 | 推流 720p@60 H.264 | 默认 `enable_stream: false` |
| 7 | MPC 与 Aimer **都升** | C2 + C3 同批 |

---

## 1. 总体路线

```
A 去 ROS ──► B 调试体系 ──► C 弹道/火控 ──► D EKF 仪表与调参 ──► E 回归上车
   (已完成)     (进行中)        (进行中)         (仪表已加)         (待排期)
```

---

## 2. 工作流与状态

| 编号 | 工作项 | 状态 | 产出 |
|---|---|---|---|
| **A** | 去 ROS 化 | ✅ 完成 | 删 `io/ros2`(6) + 3 测试；改 4 个 sentry 源 + 2 处 CMake |
| **B0** | 遥测看板 | ✅ 完成 | `tools/debug/dashboard.py`（UDP 9870 → 浏览器 9871） |
| **B1** | RTP 推流模块 | ⬜ 待做 | `tools/debug/stream_session.{hpp,cpp}` |
| **B2** | 启动脚本 + 播放页 | ⬜ 待做 | `start_streamer.py`、`res/{mediamtx.yml,playing.html}` |
| **B3** | 录制/暂停/单步控制 | ⬜ 待做 | `recorder_ctl.{hpp,cpp}`（FIFO） |
| **B4** | 画布 `draw_later` | ⬜ 待做 | `tools/debug/canvas.hpp` |
| **C1** | 弹道求解器 | ✅ 完成 | `tools/ballistics.{hpp,cpp}` |
| **C2** | Aimer 接入（可开关） | ✅ 完成 | `aimer.cpp` 走 `solve_trajectory`，失败回退解析解 |
| **C3** | Planner 参考轨迹 | ⬜ 待做 | `build_reference` + continuation（MPC 100 点） |
| **C4** | 配置段 | ⬜ 部分 | 需标定后填入 `ballistics:`（见 §4） |
| **C5** | 单测 | ✅ 完成 | `tests/ballistic_solver_test.cpp`（7 组用例） |
| **C6** | 开火延迟标定 | ⬜ 待做 | `fire_delay` / `predict_extra_delay` 实测 |
| **D** | EKF 仪表 | ✅ 仪表完成 | 新增 `nis_prefit` + innovation；门控未改 |
| **D2** | EKF 门控修正 | ⬜ 待决策 | 前哨站降级 NIS；普通车改标准 NIS + 9.488 |

---

## 3. 本轮已完成内容（可提交）

### A 去 ROS
- 删除：`io/ros2/{ros2,publish2nav,subscribe2nav}.{hpp,cpp}`、`tests/{publish,subscribe,topic_loop}_test.cpp`
- 修改：`src/sentry{,_bp,_debug,_multithread}.cpp`（移除 ROS 调用）、`CMakeLists.txt`、`io/CMakeLists.txt`
- 验证：`grep -r "rclcpp\|sp_msgs\|ament_\|rosidl\|io/ros2"` 在代码/构建文件中 **0 命中**

### B0 看板
- `tools/debug/dashboard.py`：纯标准库，UDP→HTTP→canvas 曲线；支持 `--csv`
- 本地实测：UDP 5 包 → HTTP 返回 5 样本 + 7 KB 页面

### C1/C2/C5 弹道
- `tools::solve_ballistic(v0, target, params)`：RK4 + 二次阻力，yaw 解析、pitch Newton + 兜底二分
- `Aimer::solve_trajectory`：**配置里有 `ballistics:` 段才启用**，失败自动回退 `tools::Trajectory`
- 单测：真空一致性、阻力方向、端点误差、上下坡、不可解、yaw、单调性、性能
- **数值预验证（本机无编译器，用同构 Python 复刻算法）**：端点最大误差 **0.994 mm**（阈值 2 mm）

### D EKF 仪表
- `nis_prefit`（标准 pre-fit NIS）+ 4 维 innovation 输出；4 个 debug 程序已喂给 Plotter
- 旧 `nis` 与 `recent_nis_failures` 门控**未改**，现网行为不变

---

## 4. 下一步（按顺序）

### 4.1 立即（需要你在 Linux 上验证）
```bash
cmake -S . -B build && cmake --build build -j"$(nproc)"
./build/ballistic_solver_test          # 期望 PASS 全绿
bash tests/run_outpost_tests.sh --standalone
```

### 4.2 C4/C6 标定（决定 drag 是否可信）
1. 静态落点法：3/5/8/10 m 靶纸 → 反解 `c`；
2. 高速相机定 `fire_delay`（20–40 ms）；
3. 把结果填进 `ballistics.resistance_coefficient`，先只给前哨站配置开启。

### 4.3 B1–B3 推流与录制
按 [auto_aim_upgrade_plan.md](auto_aim_upgrade_plan.md) §2 实施；GStreamer 缺失时编译降级 no-op。

### 4.4 C3 Planner 参考
`Planner::get_trajectory` 换 `build_reference`（contaimation 一阶），输出仍填 4×100 `Trajectory`。

### 4.5 D2 EKF 门控
先跑看板 30 s 抓 `recent_nis_failures` 分布，再决定：
- 前哨站 → 降级为告警（`OutpostTarget::diverged()` 已覆盖）
- 普通车 → 标准 NIS + 9.488 + YAML 可调

---

## 5. 验收门槛（每批改动都跑）

```bash
bash tests/run_outpost_tests.sh --standalone     # 4 个前哨站测试全绿
./build/ballistic_solver_test                    # 弹道全绿
cmake --build build -j"$(nproc)" --target standard sentry uav mt_standard
bash tests/run_outpost_usbcamera_tests.sh --probe-only
```
外加：录像回放 `fire_ratio`（≈0.33）不劣化、地面兵种无回归。

---

## 6. 风险登记

| # | 风险 | 影响 | 对策 |
|---|---|---|---|
| R1 | 新装甲板去灯条 → 现有 4 点 PnP 角点定义失效 | 高 | 本轮明确不接；先靠调试体系观测掉检 |
| R2 | 弹道 `c` 未标定就启用 drag | 高（系统性偏低） | C2 默认关闭，配置段显式开启；S1 标定后再开 |
| R3 | GStreamer 编码吃 CPU | 中 | 默认关闭；720p；`ultrafast`；必要时 `RTP_JPEG` |
| R4 | 求解器数值发散 | 中 | 保留解析解兜底 + 限幅 + 失败返回 false |
| R5 | NIS 门控误杀 | 中 | 已加 `nis_prefit`；先观测再改（D2） |

---

## 7. 文档索引

| 文档 | 内容 |
|---|---|
| [auto_aim_upgrade_inventory.md](auto_aim_upgrade_inventory.md) | 升级清单（U 系列、可复用资产） |
| [auto_aim_upgrade_plan.md](auto_aim_upgrade_plan.md) | 工作流细节（A/B/C 实施步骤、rmcs 组件拆解） |
| [ballistics_design.md](ballistics_design.md) | 弹道参数、标定三步法、CAN/串口通信差异 |
| [debug_and_ekf_tuning.md](debug_and_ekf_tuning.md) | 调试通道、NIS 问题、EKF 四判据、调参顺序 |
| [outpost_flow_summary.md](outpost_flow_summary.md) | 前哨站链路现状 |
| [outpost_usbcamera_test_runbook.md](outpost_usbcamera_test_runbook.md) | 外接相机测试指令 |

---

## 8. 本机无法验证的部分（需 Linux）

| 项 | 原因 |
|---|---|
| C++ 编译 | Windows 侧无 g++/cmake；已用同构 Python 验证数值 |
| GStreamer 推流 | 需 `libgstreamer1.0-dev` + mediamtx |
| 实车标定 | 需靶纸/高速相机 |
