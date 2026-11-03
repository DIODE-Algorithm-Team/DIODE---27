# 去 ROS 后的调试体系 & 卡尔曼调参指南

> 回答两个问题：**去 ROS 后怎么调**、**怎么判断 EKF 对不对**。
> 相关：[auto_aim_upgrade_plan.md](auto_aim_upgrade_plan.md)（工作流 B）、[ballistics_design.md](ballistics_design.md)、[outpost_flow_summary.md](outpost_flow_summary.md)。

---

## 0. 一句话

**去 ROS 对调试没有影响**——ROS 那套只用于「和导航交换信息」，从未参与可视化。
视觉侧原本就有 `Plotter`（UDP JSON）／`Recorder`（avi+txt）／`cv::imshow`／4 个单测；
缺的只是 **Plotter 的接收端**（它只发不收），现在补上了：`tools/debug/dashboard.py` → 浏览器实时曲线。

但有一件事必须先说：**当前的 NIS 卡方检验实现有问题，先量化再调参**（§2）。

---

## 1. 调试通道全景（去 ROS 后）

| 通道 | 代码 | 用途 | 现状 |
|---|---|---|---|
| 实时画面 | `cv::imshow`（debug 程序） | 看检测框/重投影/瞄准点 | ✅ 保留 |
| **数据看板** | `tools::Plotter` → UDP 9870 → `tools/debug/dashboard.py` | NIS / innovation / ω / 指令曲线 | ✅ **本次新增接收端** |
| 录像回放 | `tools::Recorder`（avi + txt 四元数） | 赛后复现、A/B 对比 | ✅ 保留 |
| 推流 | 工作流 B（GStreamer + mediamtx） | SSH 无桌面看画面 | 🚧 待实现 |
| 日志 | `tools::logger` → `logs/` | 事件、告警 | ✅ 保留 |
| 单元/集成测试 | `tests/outpost_*_test.cpp` | EKF 收敛、切板、快照、Aimer | ✅ 保留 |
| 调试器 | `gdb` / `lldb` | 断点、单步 | ✅ 与本项目无关，照常可用 |

启动看板：

```bash
python3 tools/debug/dashboard.py --csv run.csv      # UDP 9870 -> HTTP 9871
./build/mt_auto_aim_debug configs/sentry.yaml        # 数据源（含 NIS/innovation）
# 浏览器：http://<机器人IP>:9871/
```

看板自动分组：**NIS**（旧 `nis` 与新 `nis_prefit` 同图）、**innovation**（4 维残差）、**状态/帧率**（ω、fps、armor_num、`recent_nis_failures`）、**瞄准指令**（cmd/gimbal yaw·pitch）。

---

## 2. 先修仪表：当前 NIS 实现的两个问题

位置：[tools/extended_kalman_filter.cpp](../tools/extended_kalman_filter.cpp)

### 问题 1：算的不是 NIS（标准定义）

```cpp
x = x_add(x, K * z_subtract(z, h(x)));          // ① 先更新
Eigen::VectorXd residual = z_subtract(z, h(x)); // ② 再用“更新后”的 x 算残差
Eigen::MatrixXd S = H * P * H.transpose() + R;  // ③ 而 P 也已经是后验
```

标准 NIS 用的是 **pre-fit innovation**：`v = z − h(x_prior)`，`S = H·P_prior·Hᵀ + R`。
现在算的是 **post-fit residual**，分子被 `(I−HK)` 缩小、分母被后验 P 放大 —— 两个方向叠加，**结果不是 χ² 分布**。

### 问题 2：阈值取错了分位数

```cpp
// 卡方检验阈值（自由度=4，取置信水平95%）
constexpr double nis_threshold = 0.711;   // ← 这是 5% 分位（下界），不是 95% 上界
```

χ²(4) 的正确分位数：

| 分位 | 值 | 含义 |
|---|---|---|
| 5% | **0.711** | 下界（当前被当成上界用） |
| 50% | 3.357 | 中位数 |
| 95% | **9.488** | 上界 |
| 99% | 13.277 | 更宽松 |

所以判定式 `nis > 0.711` 在统计上等价于「**几乎每帧都判失败**」。而 [tracker.cpp:94-100](../tasks/auto_aim/tracker.cpp#L94-L100) 用 `recent_nis_failures` 累计 ≥ 40% 就把状态置 `lost`。

> 两个错误方向相反（post-fit 偏小、阈值偏严），**净效果无法先验判断** —— 这正是要先看数据的原因。

### 已经做的（安全、零行为变化）

在 [extended_kalman_filter.cpp](../tools/extended_kalman_filter.cpp) 里**新增**了标准 NIS：

```cpp
Eigen::VectorXd innovation = z_subtract(z, h(x_prior));   // pre-fit
Eigen::MatrixXd S_pre = H * P * H.transpose() + R;         // 先验 P
const double nis_prefit = innovation.transpose() * S_pre.inverse() * innovation;
```

并输出到 `data["nis_prefit"]`、`data["innovation_{yaw,pitch,distance,angle}"]`；
`mt_auto_aim_debug` / `auto_aim_test` / `uav_debug` / `sentry_debug` 已把它写进 Plotter。
**旧的 `nis` 与门控逻辑完全没动**，所以现网行为不变。

### 2.1 这套 NIS 是哪来的（sp25 与 rmcs 的情况）

| 项目 | 有没有 NIS | 实现 | 结论 |
|---|---|---|---|
| **sp_vision_25**（上游） | **有** | 与 DIODE 原版**同一份文件**（仅差 UTF-8 BOM 与结尾换行）：post-fit residual + `0.711` + 40% 窗口门控 | 这两个问题**是上游带来的**，不是 DIODE 引入；sp25 本身在赛场跑过 |
| **rmcs_auto_aim_v2** | **没有** | `ekf.hpp` 只做标准 pre-fit innovation 更新（`y = z − h(x_prior)`、`S = H·P·Hᵀ + R`），**不算 NIS、不做卡方门控**；发散靠 `xy_error` 门控（0.10/0.20 m）+ 协方差 >150 + 位置 >15 m + \|ω\|>10π + 500 ms 超时；马氏距离只用于打符（rune）关联 | 说明工业上"位置/协方差门控"已够用 |

> 这条对照很关键：**sp25 的 NIS 组合在实机上没崩**，说明 post-fit NIS 的典型值恰好落在 0.3–1 区间（两个错误部分抵消）。所以不要盲改，先量化。

### 2.2 待你决定（改门控 = 改行为）

| 方案 | 做法 | 适用 | 影响 |
|---|---|---|---|
| A（保守） | 只观察 `nis_prefit`，门控仍用旧 `nis` | 全兵种 | 零风险；但统计口径仍错 |
| B | 门控切到 `nis_prefit > 9.488`（95%） | **普通车必须走这条** | 改行为，需录像回归 |
| C | 前哨站直接**降级** NIS 门控为"只观测/告警" | 仅 `outpost` | 因为 `OutpostTarget::diverged()` 已覆盖协方差/位置/ω/超时，NIS 是冗余 |

**为什么不能一刀切**：

- 前哨站有专门的 `OutpostTarget::diverged()`（协方差 >150、\|center\|>15 m、\|ω\|>10π、NaN、500 ms 超时），NIS 门控**冗余**，可以按 rmcs 的做法降级；
- 普通车的 `Target::diverged()` **只检查 r/l 半径是否在 0.05–0.5 m**（[target.cpp](../tasks/auto_aim/target.cpp)），NIS 门控是它**唯一**的统计保护，简单删掉会让发散目标一直跟下去 → 必须走 B（修口径 + 改阈值），不能走 C。

---

## 3. 怎么判断 EKF「对不对」——4 个判据

### 判据 A：NIS 一致性（最重要）

理论：dof=4 时 NIS ~ χ²(4)，**均值 = 4**，90% 落在 **[0.711, 9.488]**。

| 现象 | 结论 | 处理 |
|---|---|---|
| NIS 均值 ≈ 4，且 90% 在区间内 | ✅ 一致 | 不用动 |
| NIS 均值 ≫ 4（如 15+） | 滤波器**过自信**：R 偏小 或 Q 偏大 | 先加 R，再降 Q |
| NIS 均值 ≪ 4（如 <1） | 滤波器**过保守**：R 偏大 或 Q 偏小 | 先减 R，再升 Q |
| NIS 周期性尖峰 | 模型与真实运动不符（如未建模的切板/加速度） | 查模型项，不要盲目加 Q |

### 判据 B：innovation 白噪声

- **零均值**：`|mean(innovation_yaw)| < 0.2 × std`（否则有系统偏差 → 标定/坐标系问题）
- **无自相关**：lag-1 自相关 < 0.3（否则 Q 偏小，滤波器"追不上"）
- 前哨站 yaw 残差若随角度呈正弦 → `kRadius` 或 `kPredictedPitch` 有偏

### 判据 C：协方差收敛

- `P` 对角应在 1–2 s 内单调下降到平台，之后小幅波动；
- 前哨站门控：[outpost_target.cpp:247-254](../tasks/auto_aim/outpost_target.cpp#L247-L254) 要求 `cov(0,0)<0.01 && cov(1,1)<0.01 && cov(4,4)<0.01` 且 `update_count ≥ 5`；
- 平台值应接近「R 与 Q 的平衡解」，若 `cov(xx)` 长期卡在 0.2，说明 Q_xy 偏大或观测太差。

### 判据 D：与真值对比（最硬）

| 场景 | 真值来源 | 判定 |
|---|---|---|
| 前哨站（旋转） | **ω = 0.8π = 2.513 rad/s 是规则给定的** | 收敛后 ω 应落在 2.51 ± 0.1，角度误差周期性但无漂移 |
| 静止靶 | 世界坐标已知 | 位置误差 < 2 cm，速度应 ≈ 0 |
| 匀速平移 | 跑道速度已知 | 速度估计误差 < 0.3 m/s |
| 仿真 | [tests/outpost_target_test.cpp](../tests/outpost_target_test.cpp) 内置真值轨迹 + 高斯噪声 | 中心 < 0.05 m、ω < 0.1 rad/s、yaw < 0.05 rad（已有断言） |

> **前哨站是最适合调 EKF 的目标**：转速是常量，相当于自带真值。建议所有调参先在它上面做完。

---

## 4. 调参顺序与量级

```
① R（观测噪声） → ② Q（过程噪声） → ③ P0（初始协方差）
```

1. **R 从测量链推**：重投影误差（px）× 像素当量（m/px）→ 方差。
   前哨站现值：`R = diag(5e-3, 5e-3, 1e-2, 1e-2)`（[outpost_target.cpp:12-20](../tasks/auto_aim/outpost_target.cpp#L12-L20)）。
   注意 **R 是方差**，若你的重投影是 3 cm，则 `R_xy ≈ 0.03² = 9e-4`，别把标准差填进去。
2. **Q 按物理写**：前哨站中心静止 → `Q_xy = 1e-6`（现在就是这么设的）；ω 允许变化 → `Q_ω = 1e-4`。
3. **P0 只影响收敛速度**，不影响稳态；调不好会表现为「前 5 帧跳变」。
4. **Q 与 dt 耦合**：Q 是「每步」噪声。换帧率（60→120 fps）时，`Q` 应随 `dt` 缩放（位置项 ∝ dt²、速度项 ∝ dt），否则同一套参数在不同帧率下表现不一致。

### 常见症状 → 处理

| 症状 | 多半是 | 处理 |
|---|---|---|
| 静止时 yaw 抖 | R_yaw 太小 | 加大 `kObservationNoiseYaw` |
| 旋转时明显滞后 | Q_ω 太小 / R 太大 | 加 `kProcessNoiseAngle`，略降 R |
| 切板瞬间飞出去 | 观测门控太松 | 收紧 `is_observation_valid`（xy 0.10/0.20 m） |
| 收敛后仍有 5 cm 偏差 | 半径/板高/外参标定 | 查 `kRadius`、`kHeightStep`、相机外参，不是 EKF 的锅 |
| 追丢后长时间不恢复 | `outpost_max_temp_lost_count` 太小 | 调大（75/150） |

---

## 5. 可视化界面：现在有什么

### 5.1 已有（本次补）

`tools/debug/dashboard.py`：**UDP 9870 → 浏览器 9871**，零依赖（Python 标准库 + 原生 canvas），可远程、可暂停、可 `--csv` 落盘。

### 5.2 待做（工作流 B）

- 视频页（WebRTC）+ 数据页合并为一个控制台：左画面右曲线；
- 录制/暂停/单步按钮驱动 `tools::Recorder`；
- 系统状态（CPU/内存/磁盘）——rmcs 的 `start-streamer` 已实现，照搬。

### 5.3 为什么不做 Qt/rqt

- rqt 是 ROS 生态，已经去掉；
- Qt 在无桌面机器上要 X11 转发，和推流方案重复；
- 浏览器方案：SSH 能开、手机能看、无需装依赖。

---

## 6. 其他需要注意的问题

| # | 问题 | 影响 | 建议 |
|---|---|---|---|
| 1 | **时间戳**：帧用「到达时刻」，不是曝光时刻 | innovation 统计被延迟污染，NIS 偏大 | 工作流 C/U7：用相机硬件时间戳 + 同步模型 |
| 2 | **单位**：Q/R 是方差，角是 rad | 填错一个数量级，EKF 直接失效 | 调参前先写一行自检 print |
| 3 | **帧率变化**：60 vs 120 fps 共用一套 Q | 高帧率下过度平滑 | Q 随 dt 缩放，或每配置单独给 |
| 4 | **参数热加载**：改 YAML 要重启 | 调参效率低 | 可加 `SIGHUP` 重读，或先接受重启（比赛期不动） |
| 5 | **录像回放的时间轴**：`auto_aim_test` 用 `t0 + t*1e6` 重建 | 与实时 dt 略有差异 | 对比实时/回放时同看 `dt` 曲线 |
| 6 | **看板自身开销**：每帧 JSON + HTTP | 极小（<1% CPU），但 UDP 丢包可接受 | 只发标量字段，不要发大数组 |
| 7 | **无桌面环境**：`cv::imshow` 会阻塞/失败 | SSH 下调试程序起不来 | 用 `xvfb-run` 或改看板模式 |
| 8 | **哨兵去 ROS 后**：`get_target_info` 不再被调用 | 该函数可能变成死代码 | 保留接口，未来接 UDP 时复用 |

---

## 7. 建议的下一步

1. **跑一次看板**，抓 30 s 前哨站数据（`--csv`），看 `nis` 与 `nis_prefit` 的真实分布；
2. 据此决定 §2 的 A/B/C（是否切门控）；
3. 再做工作流 B 的视频控制台与工作流 C 的弹道求解器。

> 调 EKF 的铁律：**先确认仪表准，再动参数**。仪表不准时，所有"调参经验"都是噪声。
