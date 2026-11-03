# 前哨站自瞄研发方案：YOLOv11-pose + 传统 ROI

> 状态：**已确定方案**（本文档）。实现进度见文末阶段表。
> 现有链路说明见 [outpost_flow_summary.md](outpost_flow_summary.md)；两者冲突时以实际代码为准。

## 1. 背景与目标

### 1.1 现状与瓶颈

现有前哨站链路：YOLO11（已是 4 关键点 pose 模型，[yolo11.cpp](../tasks/auto_aim/yolos/yolo11.cpp) L110-143）
→ 4 点 IPPE + `optimize_yaw` 三点插值 → 5 维 EKF（`OutpostTarget`）→ `Aimer` 动态攻击窗口。

三个瓶颈：

| # | 瓶颈 | 现状代码位置 |
|---|---|---|
| 1 | **6 点 PnP 的第 5/6 点是几何推算占位**，无真实图像观测，远距离测距退化压制不足 | [tracker.cpp](../tasks/auto_aim/tracker.cpp) L188 传 `{-1,-1}`；[solver.cpp](../tasks/auto_aim/solver.cpp) L138-149 内部 `world2pixel` 投影（纯几何闭环，见 flow_summary §6.3） |
| 2 | **YOLO 关键点精度受 stride 限制**（热图输出 ~2-4 px 量化误差），而 PnP 距离对灯条端点误差敏感 | [yolo11.hpp](../tasks/auto_aim/yolos/yolo11.hpp) L31，输出 50 列 = 4 xywh + 38 类 + 4 点×2 |
| 3 | **颜色仅靠类别输出**（Boutpost/Routpost/Eoutpost），无像素级复核，逆光/曝白时类别易混 | [yolo11.cpp](../tasks/auto_aim/yolos/yolo11.cpp) L139 类别直接定 color |

### 1.2 方案一句话

**YOLOv11-pose 扩展关键点（当前板 4 角 + 邻侧灯条端点）负责全帧高效检测与粗定位；
检测框内传统 CV（灰度阈值 + 轮廓 + 亚像素拟合）做端点精修与颜色复核；
精修后的真实 2D 端点喂给 6 点 PnP，替换几何占位，压制远距离测距退化。**

"精准" = 亚像素端点 + 真实 6 点观测；"高效" = 全帧只跑一次 YOLO，传统 ROI 仅处理小窗口（<1 ms）。

### 1.3 明确不做的事

- 不改 5 维 EKF 状态定义与切板逻辑（[outpost_target.cpp](../tasks/auto_aim/outpost_target.cpp) 保持不动）；
- 不改 `Aimer` / `Shooter` / 状态机（对外仍是 11 维快照）;
- 不替换普通兵种装甲板检测链路（本方案只影响 `armor.name == outpost` 分支）；
- 哨兵不打前哨站（[decider.cpp](../tasks/omniperception/decider.cpp) L143-144 过滤保持不变）。

---

## 2. 总体架构

```
io::Camera ──img,t──► YOLO11-pose（全帧一次推理，≤4 ms）
                        │  box + cls(B/R/E outpost) + 6 kpt
                        │  kpt0-3: 当前板四角   kpt4-5: 邻侧灯条上/下端
                        ▼
              OutpostRoiRefine（传统 CV，仅 box 内 ROI，≤1.5 ms）
                ├ 灰度阈值 + 轮廓 → 灯条候选
                ├ 亚像素端点拟合（直线拟合求交 / cornerSubPix）
                ├ 颜色像素级复核（R/B 通道比，兼容 extinguish）
                └ 失败 → 回退 YOLO 关键点（不阻塞链路）
                        │  Armor{points=精修四角, neighbor_pts=真实邻板端点}
                        ▼
              Solver::solve_outpost_6pt（真实 2D 端点优先，几何投影兜底）
                        ▼
              OutpostTarget 5 维 EKF（不变）→ 11 维快照（不变）
                        ▼
              Aimer（不变）→ Shooter（不变）
```

---

## 3. 关键点定义（数据集与模型 schema）

输入 640×640，输出 `[1, 4+class_num+kpt_num*3, 8400]`（**3 通道/点**：x, y, visibility）。

| kpt | 含义 | 备注 |
|---|---|---|
| 0-1 | 当前板左灯条 top / bottom | 对应 `SMALL_ARMOR_POINTS` 的左列，顺序与 [solver.cpp](../tasks/auto_aim/solver.cpp) object points 一致 |
| 2-3 | 当前板右灯条 top / bottom | 同上右列 |
| 4 | 邻侧灯条上端点 | near 侧，对应 `NeighborBarSolution` 的 near 上端（[outpost_geometry.hpp](../tasks/auto_aim/outpost_geometry.hpp) L128-204） |
| 5 | 邻侧灯条下端点 | near 侧下端 |

`kpt_num = 6`，`class_num = 38`（沿用现有类别表，含 `Boutpost/Routpost/Eoutpost`，
见 [assets/yolo11.xml](../assets/yolo11.xml) L23426 与 [armor.hpp](../tasks/auto_aim/armor.hpp) L52-65）。

- **visibility 语义**：邻板灯条经常只露一截，kpt4/5 可见性判定阈值 ≥ 0.5 才视为有效观测；
  两者任一无效 → 该帧退回现有几何占位路径（零风险灰度上线）。
- 远距小目标（>8 m）邻板端点退化时同样按 visibility 回退。

---

## 4. 传统 ROI 精修模块

新增 `tasks/auto_aim/outpost_roi.{hpp,cpp}`（header 优先，便于无 OpenVINO 单测）：

```
struct OutpostRoiResult {
  std::array<cv::Point2f, 4> armor_pts;   // 精修后的当前板四角
  std::optional<std::pair<cv::Point2f, cv::Point2f>> neighbor_pts; // 邻板灯条上/下端
  Color color;                            // 像素级颜色复核结果
  bool refined;                           // false = 全量回退 YOLO kpt
};
OutpostRoiResult refine(const cv::Mat & bgr, const Armor & yolo_out);
```

处理流程（全部在 box 外扩 30% 的 ROI 内）：

1. **灰度阈值分割**：沿用 [detector.cpp](../tasks/auto_aim/detector.cpp) 的灯条提取思路（`get_color` L279-288 的通道和判色逻辑复用为复核依据）；
2. **亚像素端点**：每根灯条取旋转矩形长边两侧像素做直线拟合（`cv::fitLine`），两端点由上下边界线与灯条轴线求交得到，精度目标 0.1-0.3 px；
3. **四角重组**：左右灯条端点按 [yolo11.cpp](../tasks/auto_aim/yolos/yolo11.cpp) L209-235 `sort_keypoints` 同样拓扑排序，与 YOLO kpt 顺序对齐后一一配对（配对距离 > 8 px 视为精修失败，回退）；
4. **颜色复核**：灯条区域内 R/B 通道和比较（同 `get_color`），与 YOLO 类别颜色不一致时以像素判定为准并打日志。

---

## 5. 具体量化目标

### 5.1 检测与精修层

| 指标 | 目标 | 测法 |
|---|---|---|
| 前哨站召回率（3-8 m，全姿态） | ≥ 99% | 测试集 + 录像回放统计 |
| 误检 | ≤ 1 帧 / 小时录像 | 连续录像回放 |
| YOLO 关键点误差（模型原始输出） | RMSE ≤ 2.0 px（640 输入尺度） | 测试集标注真值 |
| ROI 精修后端点误差 | ≤ 0.5 px | 高分辨率截图人工真值 |
| 颜色判定准确率 | ≥ 99.5%（含 extinguish） | 测试集混淆矩阵 |
| YOLO 推理耗时 | ≤ 4 ms（INT8，目标设备） | `auto_aim_test` 计时 |
| ROI 精修耗时 | ≤ 1.5 ms / 帧 | 单元测试计时 |
| 整链路耗时（detect→solve→track→aim） | ≤ 10 ms | `auto_aim_test` 分段计时 |

### 5.2 解算与跟踪层（PR-4 验收）

| 指标 | 目标 | 说明 |
|---|---|---|
| 重投影误差 | 较阶段 0 基线下降 ≥ 30% | 6 点真实观测 vs 几何占位，同批录像 A/B |
| 5 m 处距离误差 | ≤ 2%（基线先测，目标收紧 30%） | 与激光测距/标定真值对比 |
| EKF 稳态 ω 误差 | ≤ 0.05 rad/s | 现门限 0.1（[outpost_target_test.cpp](../tests/outpost_target_test.cpp) L1-14），收紧一半 |
| EKF 收敛帧数 | ≤ 10 帧 | 现有收敛门控 ≥5 帧基础上统计 |
| 真实端点使用率 | 邻板可见帧中 ≥ 90% 帧走真实观测（非回退） | 日志统计 |

### 5.3 实弹层（PR-6 验收）

| 指标 | 目标 |
|---|---|
| 前哨站单次击毁耗时 | 较现有版本缩短 ≥ 20%（同距离同转速对比） |
| 开火命中率 | 可测并有记录（弹丸记数 / 击打反馈） |

---

## 6. 分阶段实施

| 阶段 | 内容 | 主要改动 | 验收 |
|---|---|---|---|
| **0 基线测量** | 用 `auto_aim_test` 回放现有前哨站录像，统计重投影误差/距离误差/分段耗时基线 | 无代码改动，产出基线报告 | 基线数据入库（docs 附录） |
| **PR-1 数据与标注** | 标注规范（§3 schema）+ 标注工具对接；重点采集切板瞬间、逆光曝白、运动模糊、5-8 m 小目标；≥ 2000 帧前哨站关键点标注 | 数据集仓库（不入本仓库） | 标注抽检合格率 ≥ 98% |
| **PR-2 模型训练导出** | yolov11n-pose（6 kpt）训练 → ONNX → OpenVINO INT8 导出 `assets/yolo11_outpost_pose.xml`；[yolo11.cpp](../tasks/auto_aim/yolos/yolo11.cpp) 关键点数配置化（`kpt_num` 从 YAML 读，默认 4 兼容旧模型） | yolo11.cpp / yolo11.hpp / configs/*.yaml 新增 `yolo11_outpost_pose_model_path` | §5.1 前三行达标 |
| **PR-3 ROI 精修模块** | 新增 [outpost_roi.hpp](../tasks/auto_aim/outpost_roi.hpp) / .cpp；在 `YOLO11::detect` 出口对 outpost 装甲板调用精修；失败回退 | 新文件 + [CMakeLists.txt](../CMakeLists.txt) 测试目标 | 新增 `outpost_roi_test` 全过 + 耗时达标 |
| **PR-4 真实端点接入 6 点 PnP** | `Armor` 增加邻板端点字段；[tracker.cpp](../tasks/auto_aim/tracker.cpp) L299-345 前哨站分支把精修端点传入 `solve_outpost_6pt`（替换 `{-1,-1}` 占位）；[solver.cpp](../tasks/auto_aim/solver.cpp) L94-199 真实端点优先、几何投影兜底 | tracker.cpp / solver.cpp / armor.hpp | §5.2 全部达标 |
| **PR-5 基准与回归** | `auto_aim_test` 增加分段计时与重投影误差统计输出；扩展 [outpost_target_test.cpp](../tests/outpost_target_test.cpp) 真实端点注入场景；Windows 等价工程（out/win_outpost_tests）同步 | tests/* | 一键脚本全绿 |
| **PR-6 实车验证** | kRadius 重新标定（现 0.2765 vs rmcs 0.275，见 flow_summary §6.4）；EKF R 矩阵按新观测噪声更新；实弹对比测试 | outpost_const.hpp / outpost_target.cpp / configs | §5.3 达标 |

依赖关系：PR-2 → PR-3 → PR-4 → PR-5 → PR-6；PR-1 与阶段 0 可并行启动。
PR-3/PR-4 之间保持可独立回退：任何精修失败帧自动走现有几何占位路径，链路不中断。

---

## 7. 测试与验收方式

1. **单测**：复用一键脚本 [tests/run_outpost_tests.sh](../tests/run_outpost_tests.sh)（Windows 用
   [tests/run_outpost_tests.ps1](../tests/run_outpost_tests.ps1)），新增 `outpost_roi_test` 加入目标列表；
2. **离线回放**：`./build/auto_aim_test -c configs/demo.yaml -s 0 -e 0 assets/demo/demo`，
   前哨站录像走新链路，对比阶段 0 基线；
3. **A/B 开关**：configs 增加 `outpost_roi_refine: true/false`，同一录像一键切换对比，
   避免新旧实现混杂导致的统计失真；
4. **实弹**：同转速（0.8π）、同距离（3/5/7 m 档）下新旧版本各打 3 组，记录击毁耗时与命中率。

---

## 8. 风险与备选

| 风险 | 影响 | 对策 |
|---|---|---|
| 邻板灯条长期只露极少边缘，kpt4/5 学不出 | 真实端点使用率低 | visibility 回退保证基线性能；数据集专项采集切板序列；必要时 schema 降级为 4 kpt + 仅 ROI 找邻板端点 |
| INT8 量化掉点（关键点对量化敏感） | §5.1 指标不达标 | FP16 备选模型；或对关键点头做 QAT |
| 远距（>8 m）ROI 内灯条仅数像素，传统精修失效 | 端点误差超标 | 回退 YOLO kpt + EKF 平滑；精修模块加 ROI 面积下限门控 |
| Windows 开发机跑不了 OpenVINO 全链路 | 联调受限 | 算法层用 out/win_outpost_tests 等价工程（见 flow_summary §5.6 注）；推理层在 Linux 上位机验证 |
| 颜色复核与类别冲突频率过高 | 日志噪音 / 误修正 | 冲突率 > 2% 时先只记录不修正，人工分析后再启用修正 |

---

## 9. 阶段进度

- [ ] 阶段 0：基线测量
- [ ] PR-1：数据与标注（≥2000 帧）
- [ ] PR-2：yolov11n-pose 6 kpt 训练 + INT8 导出
- [ ] PR-3：ROI 精修模块 + `outpost_roi_test`
- [ ] PR-4：真实端点接入 6 点 PnP
- [ ] PR-5：基准与回归
- [ ] PR-6：实车验证与标定更新
