# DIODE---27 项目索引（DSH 整理）

27赛季自瞄，基于 sp25 开发。

> 本文件是 **AI 助手（DeepSeek Harness）整理的项目索引**，不影响上游原版文档。
> 上游 `sp_vision_25` 的完整说明（含「轨迹视角下的自瞄理论」）仍在 [readme.md](readme.md)。

---

## 前哨站（Outpost）测试

外接相机跑前哨站自瞄的命令、配置与排查手册：

- 指令手册：[docs/outpost_usbcamera_test_runbook.md](docs/outpost_usbcamera_test_runbook.md)
- 流程与代码位置：[docs/outpost_flow_summary.md](docs/outpost_flow_summary.md)
- 设计/迁移方案：[docs/outpost_migration.md](docs/outpost_migration.md)
- 前哨站视觉方案（YOLO-pose，已挂起）：[docs/outpost_yolo_pose_plan.md](docs/outpost_yolo_pose_plan.md)

```bash
# 算法层（无需相机/下位机/OpenVINO）
bash tests/run_outpost_usbcamera_tests.sh --standalone

# 环境 + 设备探测（相机、CAN、串口）
bash tests/run_outpost_usbcamera_tests.sh --probe-only

# 外接相机实时识别（弹窗）
bash tests/run_outpost_usbcamera_tests.sh --detect --display
```

---

## 自瞄升级

**主计划（从这里开始）**：[docs/plan_master.md](docs/plan_master.md) —— 工作流 A–D 的状态、下一步、验收门槛。

细节文档：

- 升级清单（可复用资产 / 优先级 / 审查问题）：[docs/auto_aim_upgrade_inventory.md](docs/auto_aim_upgrade_inventory.md)
- 工作流细节（去 ROS / 调试体系 / 弹道实施步骤）：[docs/auto_aim_upgrade_plan.md](docs/auto_aim_upgrade_plan.md)
- 弹道参数与标定（Q4 详细方案）：[docs/ballistics_design.md](docs/ballistics_design.md)
- 调试体系与 EKF 调参：[docs/debug_and_ekf_tuning.md](docs/debug_and_ekf_tuning.md)

当前进度一句话：去 ROS ✅、遥测看板 ✅、弹道求解器 + Aimer 接入 ✅（可开关）、EKF 仪表 ✅；推流、Planner 参考、实车标定待做。

---

## 调试看板（替代 ROS 可视化）

```bash
# 终端 1：数据看板（UDP 9870 -> 浏览器 9871，纯标准库）
python3 tools/debug/dashboard.py --csv run.csv

# 终端 2：自瞄进程（mt_/uav_/sentry_debug 会把 NIS、innovation 发到 9870）
./build/mt_auto_aim_debug configs/sentry.yaml

# 浏览器打开 http://<机器人IP>:9871/
```

关键字段：`nis`（旧口径，post-fit）、`nis_prefit`（标准 NIS，dof=4 应在 0.711–9.488）、
`innovation_*`（4 维残差）、`recent_nis_failures`、`w`（ω）、`fps`。

---

## 编译与验证

> 2026-11-03 已在 Ubuntu 24.04 / GCC 13.3 上完成全量构建与离线测试，结果见
> [docs/verification_report.md](docs/verification_report.md)（含环境、修复项、日志与未验证部分）。

```bash
cmake -S . -B build && cmake --build build -j"$(nproc)"

./build/ballistic_solver_test                  # 弹道求解器单测
bash tests/run_outpost_tests.sh                # 前哨站 4 个测试（复用 build/）
bash tests/run_outpost_tests.sh --standalone   # 不依赖 OpenVINO 的独立编译
```

---

## 参考

- 上游完整说明：[readme.md](readme.md)（27 KB，含 §4 轨迹视角下的自瞄理论）
- 参考工程：`rmcs_auto_aim_v2/`、`sp_vision_25-main/`
- 弹道算法推导：`弹道解算及云台角度求解算法.txt`
