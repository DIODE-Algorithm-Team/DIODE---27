# DIODE-27 自瞄升级规划（去 ROS + 调试体系 + 弹道）

> 输入约束（2026-11-02 确认）：
> 1. 所有产出文件必须落在 `D:\RM\DIODE-27` 内；
> 2. 装甲板更新（去除灯条）相关**不做**；
> 3. 采用 `rmcs_auto_aim_v2` 的调试方案；
> 4. 项目**去 ROS 化**。
>
> 关联：[auto_aim_upgrade_inventory.md](auto_aim_upgrade_inventory.md)（升级清单）、[outpost_flow_summary.md](outpost_flow_summary.md)（现状链路）、[outpost_usbcamera_test_runbook.md](outpost_usbcamera_test_runbook.md)（测试指令）。

---

## 0. 四条约束的落地解读

| # | 你的要求 | 本规划的落地 | 影响范围 |
|---|---|---|---|
| 1 | 产出在 `DIODE-27` 内 | 所有新文件只写这四个目录：`docs/`、`tools/`、`tasks/`、`tests/`；脚本与配置进 `tests/`、`configs/`、`tools/debug/res/`。**不再引用 `D:\RM\` 下的外部副本** | 全局 |
| 2 | 装甲板更新/去灯条 **不做** | 清单里的 **U1（邻侧灯条真实端点）、U5 灯条部分、YOLO-pose 的 kpt4/kpt5** 全部移出本轮；4 点/6 点 PnP 的现有几何路径保持原样 | 前哨站视觉 |
| 3 | 用 rmcs 的调试方案 | 移植**推流 + 浏览器控制台 + 录制/暂停/单步 + 系统状态**；放弃其 ROS 可视化与 shm 双进程（见 §2） | 新模块 `tools/debug/` |
| 4 | 去 ROS 化 | 删除 `io/ros2/`、3 个 ROS 测试、CMake 的条件编译块；`sentry*` 三处 ROS 调用移除或换成 UDP（见 §1） | `io/`、`src/`、`CMakeLists.txt` |

> **决策门（需你确认，§7-Q1）**：若 27 赛季装甲板**物理上取消了灯条**，则现有 4 点 PnP 的角点定义（取自灯条上下端点）与传统阈值检测链路都会失效。这与约束 2 直接冲突——"不做灯条更新"与"灯条没了"是两件事。本规划按「**不做任何灯条相关改动**」执行；若真的去灯条，需要单独立项（神经网络直接回归四角/pose），不混在本轮。

---

## 1. 工作流 A：去 ROS 化（P0，**已完成** 2026-11-02）

> 状态：✅ 代码删除与 CMake 改造已执行；`grep` 验证无残留（仅文档提及）。待 Linux 侧 `cmake` 编译确认。

### 1.1 现状依赖地图（已核查）

| 文件 | ROS 用途 | 处理 |
|---|---|---|
| `io/ros2/ros2.{hpp,cpp}` | 封装节点 + 双线程 spin | ✅ 已删除 |
| `io/ros2/publish2nav.{hpp,cpp}` | 发布目标信息（`std_msgs::String`）给导航 | ✅ 已删除（Q2：导航与自瞄无信息传递） |
| `io/ros2/subscribe2nav.{hpp,cpp}` | 订阅 `EnemyStatusMsg` / `AutoaimTargetMsg`（`sp_msgs`） | ✅ 已删除 |
| [src/sentry.cpp](../src/sentry.cpp) | 取无敌甲列表 + 发布目标 | ✅ 已移除 |
| [src/sentry_bp.cpp](../src/sentry_bp.cpp) | 同上 | ✅ 已移除 |
| [src/sentry_debug.cpp](../src/sentry_debug.cpp) | 同上（含 `get_auto_aim_target` 调用） | ✅ 已移除 |
| [src/sentry_multithread.cpp](../src/sentry_multithread.cpp) | 同上 | ✅ 已移除 |
| `tests/publish_test.cpp`、`subscribe_test.cpp`、`topic_loop_test.cpp` | ROS 通信测试 | ✅ 已删除 |
| [io/CMakeLists.txt](../io/CMakeLists.txt) | `find_package(ament_cmake...)` + 条件 `target_sources` | ✅ 已删整段 |
| [CMakeLists.txt](../CMakeLists.txt) | ROS 探测 + 条件编译 sentry 系列 | ✅ 改为无条件编译 `sentry*` |
| `tasks/omniperception/decider.*` | **无 ROS 依赖**（只吃 `std::vector<int8_t>`） | 不改 |

结论：ROS 面**完全被隔离在 `io/ros2/` + 4 个 sentry 源 + 3 个测试 + 2 处 CMake**，没有渗透到 `tasks/`、`tools/`、`io/camera`、`io/cboard`。去 ROS 是低风险机械工作。

### 1.2 实施步骤（已执行）

```bash
# A1 删除 ROS 实现与测试                       ✅
rm -rf io/ros2
rm tests/publish_test.cpp tests/subscribe_test.cpp tests/topic_loop_test.cpp

# A2 清理 CMake（两处）                         ✅
#   CMakeLists.txt  删 ROS 探测块，sentry 系列无条件编译
#   io/CMakeLists.txt 删 ROS 条件块

# A3 清理 sentry 四处 ROS 调用（sentry / sentry_bp / sentry_debug / sentry_multithread）  ✅
```

### 1.3 `sentry` 的功能降级（Q2 已确认：不需要替代）

| 功能 | 原实现 | 处理 |
|---|---|---|
| 取「无敌甲」编号 → 过滤目标 | `ros2.subscribe_enemy_status()` → `decider.get_invincible_armor()` | **已直接删除**（Q2 确认：导航与自瞄之间无信息传递）。`Decider::get_invincible_armor()` 接口保留，未来若有 UDP/裁判系统输入可直接喂入 |
| 把目标信息发给导航（哨兵追击） | `ros2.publish(decider.get_target_info(...))` | **已直接删除**，哨兵退化为「原地自瞄」。`Decider::get_target_info()` 接口保留 |

> 不新增 `io/nav/nav_udp`——Q2 已明确没有信息传递需求，避免留死代码。

### 1.4 验收（可执行）

```bash
# 1) 全仓无 ROS 关键字（排除两个参考仓库）
grep -rn "rclcpp\|sp_msgs\|ament_\|rosidl" --include=*.cpp --include=*.hpp --include=*.txt \
  --exclude-dir=sp_vision_25-main --exclude-dir=rmcs_auto_aim_v2 --exclude-dir=build --exclude-dir=out . \
  && echo "仍有残留" || echo "ROS 清零 OK"

# 2) 无 ROS 环境下配置 + 编译
cmake -S . -B build_noROS -DCMAKE_BUILD_TYPE=Release
cmake --build build_noROS -j"$(nproc)" --target standard sentry uav mt_standard

# 3) 二进制不链接 ROS
ldd build_noROS/standard build_noROS/sentry | grep -i "ros\|rclcpp" && echo "仍链接 ROS" || echo "OK"
```

**通过标准**：1) 无输出；2) 全部编译成功；3) `sentry` 目标在无 ROS 机器上首次可编译。

**风险**：哨兵失去无敌甲过滤与追击发布（功能降级，需队伍确认可接受）；回退 = 保留当前分支 tag。

---

## 2. 工作流 B：调试体系移植（P0.5，与 A 并行）

> ✅ **B0 已完成（2026-11-02，不依赖 GStreamer）**：`tools/debug/dashboard.py` —— 收 `Plotter` 的 UDP JSON，浏览器实时曲线
> （NIS / innovation / ω / 指令），支持 `--csv` 落盘。同时给 EKF 增加标准 `nis_prefit` 观测字段（不改门控）。
> 详见 [debug_and_ekf_tuning.md](debug_and_ekf_tuning.md)。

### 2.1 rmcs 方案拆解与可移植性

| 组件 | 位置 | ROS 依赖 | 移植决定 |
|---|---|---|---|
| **RTP 推流** `StreamSession` | `src/module/debug/visualization/stream_session.*` | 无（GStreamer） | ✅ **移植**（B1） |
| **WebRTC 网关 + 播放页** | `tool/res/{mediamtx.yml,playing.html,start-streamer}`、`tool/install-server.sh` | 无（Python + mediamtx） | ✅ **移植**（B2/B3） |
| **录制/暂停/单步 FIFO 控制** | `start-streamer` 的 `/api/record`、`/api/play_pause`、`/api/step_*` + `/tmp/autoaim/*` | 无 | ✅ **移植**（B4） |
| **系统状态面板**（CPU/负载/内存/磁盘/进程占用） | `start-streamer` 的 `_read_system_status` | 无 | ✅ **移植**（随 B2） |
| **`draw_later` 画布** | `src/kernel/visualization.hpp`、`utility/image/drawable.*` | 绘制部分无；`publish()` 是 ROS visual | ⚠️ **裁剪移植**（B5，只保留画布） |
| **Feishu shm 双进程** | `kernel/feishu.hpp`、`utility/shared/interprocess.hpp` | 无 | ❌ **不移植**（过重；用 CSV/JSON + 现有 UDP Plotter 替代） |
| **ros2 publish 可视化** | `kernel/visualization.hpp::publish` | 有 | ❌ **删除** |

### 2.2 目标架构（去 ROS 版）

```
自瞄进程 (standard/uav/sentry/mt_auto_aim_debug)
  ├─ Canvas（draw_later）→ 一帧只画一次
  ├─ StreamSession ──RTP/H264──► 127.0.0.1:5000
  │                                  │
  │                            mediamtx（WebRTC 网关 :8889）
  │                                  │
  │                    浏览器 http://<ip>:18080/autoaim → playing.html
  │                                  │  POST /api/record
  ├─ RecorderCtl ◄──FIFO /tmp/diode/recorder/{start,stop}──┘
  │        └─► tools::Recorder → assets/records/<时间>_<兵种>.avi + .txt
  └─ Plotter（已有）──UDP JSON──► 127.0.0.1:9870（离线曲线工具，保持不变）
```

### 2.3 实施步骤

| 步骤 | 产出 | 说明 |
|---|---|---|
| **B1** | `tools/debug/stream_session.{hpp,cpp}` | GStreamer `appsrc ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink`；`push_frame(const cv::Mat&)`；编译期 `HAS_GSTREAMER` 检测，缺失时降级 no-op（不阻塞比赛构建） |
| **B2** | `tools/debug/start_streamer.py` | 由 rmcs `start-streamer` 改造：进程名 `rmcs_executor` → `standard\|uav\|sentry` 可配；安装根 `/opt/autoaim` → `<repo>/tools/debug/`（免 root）；端口沿用 18080/8889 |
| **B3** | `tools/debug/res/{mediamtx.yml,playing.html}` | 原样复用，只改标题与路径 |
| **B4** | `tools/debug/recorder_ctl.{hpp,cpp}` | 非阻塞轮询 FIFO，控制现有 [tools/recorder.hpp](../tools/recorder.hpp)；写 context 文件供播放页读状态 |
| **B5** | `tools/debug/canvas.hpp` | `draw_later(Text/Area/Point/Line/ArmorShape)` + `flush(img)`；**先只用在 debug 程序**（`mt_auto_aim_debug`、`uav_debug`、`sentry_debug`），主程序不动 |
| **B6** | 配置开关 | `debug: {enable_stream, framerate, monitor_host, monitor_port, stream_type}` 全部带缺省（`enable_stream: false`），缺键不抛异常 |

### 2.4 与现有调试手段的关系

| 现有 | 处理 |
|---|---|
| `cv::imshow`（20+ 处，多在 debug 程序与 detector 内部） | **保留**为本地回退；debug 程序改为 `--stream` 时代替 imshow。detector 内的 imshow 由 `YOLO(config, debug)` 的 `debug` 参数控制，已默认关闭，不动 |
| `tools::Plotter`（UDP JSON → 9870） | **保留**，与推流互补（曲线 vs 画面） |
| `tools::Recorder`（avi + txt） | **保留**，由 B4 增加 FIFO 控制 |
| `logs/*.screenlog` | 保留 |

### 2.5 依赖与验收

```bash
# 依赖（一次性）
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                    gstreamer1.0-plugins-ugly gstreamer1.0-libav

# 启动（两条命令）
python3 tools/debug/start_streamer.py          # 拉起 mediamtx + 播放页
./build/mt_auto_aim_debug configs/sentry.yaml   # 自瞄进程推流

# 浏览器打开
#   http://<机器人IP>:18080/autoaim
```

**验收清单**：
1. SSH 无桌面环境，浏览器能看到 60 fps 带标注的画面（延迟 < 200 ms）；
2. 页面点「录制」→ `assets/records/` 下生成 `.avi` + `.txt`（时间戳、四元数），点「停止」正常收尾；
3. 页面显示 CPU/内存/磁盘/进程占用，数值与 `htop` 一致；
4. 自瞄进程 `kill -9` 后 mediamtx 被自动回收（`PR_SET_PDEATHSIG`）；
5. 未安装 GStreamer 的机器上 `cmake` 仍能通过，推流自动降级为 no-op。

---

## 3. 工作流 C：弹道升级（P1，去掉灯条项后的核心）

> U1/U5-灯条已按约束 2 移出，剩余全部是「弹道 + 火控」，不碰识别。
> **参数、标定、通信影响的完整方案见 [ballistics_design.md](ballistics_design.md)**（Q4 的答复，已含实测数据表）。

| 步骤 | 内容 | 落点 | 工作量 |
|---|---|---|---|
| **C1** | `tools/ballistics.{hpp,cpp}`：RK4 + 二次阻力 + `(ψ, ϑ, τ)` 三未知量 Newton（解析灵敏度 18 维） | 新文件 | 2–3 人日 |
| **C2** | `Aimer` 接入：替换 `tools::Trajectory`，解析解保留兜底 | [aimer.cpp:98](../tasks/auto_aim/aimer.cpp#L98)、[:127](../tasks/auto_aim/aimer.cpp#L127) | 1 人日 |
| **C3** | `Planner::get_trajectory` 接入 `build_reference`（continuation） | [planner.cpp:179-203](../tasks/auto_aim/planner/planner.cpp#L179-L203) | 3–4 人日 |
| **C4** | 配置 `ballistics:` 段（物理四项 + `c` 覆盖 + 双延迟 + 枪口偏心），全部带缺省；老配置零变化 | `configs/*.yaml` | 0.5 人日 |
| **C5** | `tests/ballistic_solver_test.cpp`：3/5/8/12/20 m × 静止/平移/旋转 × 4096 步 RK4 参考 | 新测试 | 1 人日 |
| **C6** | 预测延迟统一标定（readme §4.4 实测 ≈15 ms） | `high/low_speed_delay_time` | 0.5 人日 |

**验收指标**（对齐 txt 的 benchmark）：`e_miss ≤ 2.2 mm`、P99 耗时 ≤ 6.5 ms；与现解析解对比远距 pitch 差；4 个既有 outpost 测试全绿。

---

## 4. 明确不做（本轮）

| 项 | 原因 |
|---|---|
| U1 邻侧灯条真实 2D 端点 | 约束 2 |
| U5 的灯条亚像素精修（角点精修也暂缓） | 约束 2 |
| YOLO-pose 的 kpt4/kpt5（邻侧灯条端点） | 依赖灯条，与约束 2 冲突 |
| `outpost_yolo_pose_plan.md` 的整体落地 | 同上，挂起 |
| Feishu shm 双进程、ROS visual publish | 约束 3/4（过重/依赖 ROS） |
| 能量机关（Rune）相关 | 不在自瞄升级范围 |
| 枪口偏心（U8）、时钟同步（U7） | 需实车标定，排到 P2 |

---

## 5. 里程碑（4 周）与验收矩阵

| 周 | 工作流 A（去 ROS） | 工作流 B（调试） | 工作流 C（弹道） |
|---|---|---|---|
| W1 | A1–A3 代码删除、CMake 改造、sentry 降级 | B1 推流模块骨架 + B6 配置 | C1 求解器 + C5 单测骨架 |
| W2 | A4 验证（无 ROS 构建 + ldd）、文档 | B2/B3 播放页 + 启动脚本 | C2 Aimer 接入、C6 延迟标定 |
| W3 | — | B4 录制 FIFO + B5 画布 | C3 Planner 参考 |
| W4 | 回归（前哨站 4 测试 + 地面录像） | 全链路验收（§2.5 五条） | 回归 + 指标对比报告 |

**统一门槛**（每周五跑）：
```bash
bash tests/run_outpost_tests.sh --standalone          # 4 个前哨站测试全绿
bash tests/run_outpost_usbcamera_tests.sh --probe-only # 环境无回归
# 无 ROS 构建
cmake -S . -B build_noROS && cmake --build build_noROS -j"$(nproc)" --target standard sentry uav
```

---

## 6. 风险登记

| # | 风险 | 概率 | 影响 | 对策 |
|---|---|---|---|---|
| R1 | 27 赛季装甲板真的去灯条 → 4 点 PnP 角点定义失效 | 中 | 高 | §7-Q1 先确认；若成立则单独立项，本轮不接 |
| R2 | 哨兵去 ROS 后失去无敌甲过滤/追击发布 | 高（必然） | 中 | 先方案 ①，保留 `io::Nav` 接口，后续 UDP 补回 |
| R3 | GStreamer 编码吃 CPU，影响自瞄帧率 | 中 | 中 | 默认 `enable_stream: false`；比赛模式关闭；`x264enc tune=zerolatency speed-preset=ultrafast`；必要时切 `RTP_JPEG` |
| R4 | 弹道求解器数值发散 | 中 | 中 | 保留解析解兜底 + 迭代上限 + 限幅（txt 已给回退策略） |
| R5 | 现有 `Plotter`/`Recorder` 与新的 FIFO 控制冲突 | 低 | 低 | B4 只读写 FIFO/context，不改 `Recorder` 内部 |
| R6 | 删除 ROS 测试后覆盖率下降 | 低 | 低 | 无所谓（这些测试只验证 ROS 通道本身） |

---

## 7. 决策记录（2026-11-02 已确认）

| # | 问题 | 结论 | 对规划的影响 |
|---|---|---|---|
| Q1 | 装甲板/灯条 | **物理上没有灯条了**，但**优先跑通**，识别仍走原链路（现有模型/现有 4 点 PnP 不改） | 约束 2 维持：不碰识别；`outpost_yolo_pose_plan.md` 继续挂起。**风险 R1 转为已知缺口**：新板在真机上可能掉检，用调试体系（工作流 B）先观测，不阻塞 A/B/C |
| Q2 | 导航↔自瞄通信 | **没有信息传递**（确定） | 不写 UDP 替代，直接删；`Decider` 接口保留 |
| Q3 | 推流参数 | **1280×720@60 H.264** | B1/B2 默认值固定 |
| Q4 | 弹道参数 | 需要**详细方案**（见 [ballistics_design.md](ballistics_design.md)）；自瞄与电控（云台）有通信 | C4 参数化按该文档执行；命令通道区分 CAN / 串口 |
| Q5 | MPC | **都升**（Aimer + Planner 一起） | C2 与 C3 同批交付，不再分阶段 |

> 项目自信：`sp_vision_25-main`、`rmcs_auto_aim_v2` 均已在 Linux 实机与赛场验证。本规划的改动全部是「加新模块 + 删死代码 + 换内部实现」，不推翻既有架构，按现有测试与录像回放即可回归。

---

## 8. 产出文件约定（约束 1 的落地）

本轮所有文件只会写在这些位置：

```
DIODE-27/
├─ docs/
│   ├─ auto_aim_upgrade_inventory.md      已有（升级清单）
│   ├─ auto_aim_upgrade_plan.md           本文（规划）
│   └─ ...                               后续实施记录
├─ tools/
│   ├─ ballistics.{hpp,cpp}              C1
│   └─ debug/                             B1–B5
│       ├─ stream_session.{hpp,cpp}
│       ├─ recorder_ctl.{hpp,cpp}
│       ├─ canvas.hpp
│       ├─ start_streamer.py
│       └─ res/{mediamtx.yml,playing.html}
├─ tasks/auto_aim/
│   ├─ aimer.cpp                          C2（改）
│   └─ planner/planner.cpp                C3（改）
├─ tests/
│   └─ ballistic_solver_test.cpp          C5
└─ configs/
    └─ *.yaml                             B6/C4（加键，全部带缺省）
```

**不会**再引用 `D:\RM\rmcs_auto_aim_v2`、`D:\RM\sp_vision_25-main` 等外部副本；需要参考的代码会**复制并改写**进上述目录。
