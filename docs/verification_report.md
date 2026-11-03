# DIODE-27 代码可行性验证报告

> 验证日期：2026-11-03（Asia/Shanghai）
> 验证环境：Ubuntu 24.04.5 LTS x86_64 / GCC 13.3.0 / CMake 3.28.3 / 8 核
> 代码状态：`git` 工作区（含弹道、前哨站、去 ROS、EKF 仪表等未提交改动）
> 产物位置：`build/`（编译产物）、`out/logs/`（本次全部日志）、`out/*.yaml`（验证用临时配置）

---

## 1. 结论

代码在 Linux 上**可以完整编译并运行离线算法链路**，但首次验证时被 6 类阻断/兼容性问题卡住，均已修复：

| 结论项 | 结果 |
|---|---|
| 全量构建 | ✅ 48 个 CMake 目标全部编译/链接成功，0 error |
| 弹道求解器单测 | ✅ 134 项断言通过，0 失败 |
| 前哨站 4 个测试（CMake 模式） | ✅ 41 + 22 + 28 + 7323–7329 项通过（随帧时序浮动），0 失败 |
| 前哨站 4 个测试（standalone） | ✅ 41 + 22 + 28 + 7329 项通过，0 失败 |
| 启用阻力弹道的 Aimer 链路 | ✅ 7328–7332 项通过，0 失败（`c=0.0201`） |
| YOLO/OpenVINO 冒烟 | ✅ demo 视频推理正常，低阈值下 199 帧中 184 帧检出装甲板 |
| 完整自瞄流水线冒烟 | ✅ `auto_aim_test` 跑 250 帧不崩溃 |
| 动态库完整性 | ✅ 所有可执行文件 `ldd` 无 `not found` |
| 去 ROS 检查 | ✅ 源码/构建脚本中 `rclcpp/sp_msgs/ament_/rosidl/io/ros2` 0 命中 |
| 实机链路（相机/CAN/串口/云台） | ⛔ 本机无设备，未验证（见 §6） |

**一句话**：离线算法与构建链路可行；上车前必须补外接相机、CAN/串口和实弹标定验证。

---

## 2. 验证环境与依赖

| 依赖 | 版本 / 位置 |
|---|---|
| OS / 内核 | Ubuntu 24.04.5 LTS，`7.2.10` x86_64 |
| 编译器 | g++ 13.3.0（Ubuntu 13.3.0-6ubuntu2~24.04.1） |
| CMake | 3.28.3 |
| OpenCV | 4.6.0（含 contrib/aruco） |
| Eigen3 | 3.4.0 |
| fmt / spdlog | 9.1.0 / 1.12.0 |
| yaml-cpp / nlohmann_json | 0.8.0 / 3.11.3 |
| Ceres | 2.2.0 |
| OpenVINO | 系统包 2024.6.0（`/usr/lib/cmake/openvino2024.6.0`）+ Python wheel 2026.1（`/home/gzy/openvino_env/...`） |

> 本次实际链接的是 wheel 里的 `libopenvino.so.2610`（见 §5.3 的 RUNPATH 说明）；
> 系统 2024.6 也已安装，可用 `-DOpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0` 切换。

---

## 3. 首次验证遇到的问题与修复

### 3.1 CMake 配置失败：目标重复 + 未定义变量

**现象**：`cmake -S . -B build` 在 150 行报

```
add_executable cannot create target "ballistic_solver_test" because another
target with the same name already exists.
```

**根因**：`CMakeLists.txt` 末尾重复定义了一次 `ballistic_solver_test`，并引用了从未设置的
`${REPO_ROOT}`；同时文件末尾缺少换行。第一次定义（L134）已经通过 `tools` 目标获得
`ballistics.cpp/trajectory.cpp`，末尾重复定义既冲突又会导致符号重复。

**修复**：删除末尾重复块，只保留 L134 的定义。配置/编译通过。

### 3.2 `calibrate_camera` 在 OpenCV 4.6 + yaml-cpp 0.8 下编译失败

**现象**：

```
error: 'class YAML::Node' has no member named 'value'
error: 'cv::aruco::CharucoBoard' has not been declared
error: 'interpolateCornersCharuco' is not a member of 'cv::aruco'
```

**根因**：

- `yaml.value(key, default)` 不是 yaml-cpp 的 API（疑似误用 OpenCV `FileStorage` 写法）；
- OpenCV 4.7 才把 ChArUco 合并进 `objdetect`；4.6 的 API 在
  `opencv2/aruco/charuco.hpp`，且 3D 角点字段是 `chessboardCorners` 而不是 `objPoints`。

**修复**（`calibration/calibrate_camera.cpp`）：

- 用 `yaml[key] ? yaml[key].as<T>() : default` 替代 `.value()`；
- 对 `< 4.7` 条件包含 `aruco/charuco.hpp`，并用 `board->chessboardCorners`；
- 对 `>= 4.7` 使用 `board->getChessboardCorners()`，保持前后兼容。

### 3.3 `auto_aim_test` 运行时段错误（真实缺陷）

**现象**：`./build/auto_aim_test ...` 首帧附近 `SIGSEGV`，gdb 栈：

```
#0 auto_aim::Target::diverged() const
#1 auto_aim::Tracker::track(...)
#2 main
```

**根因**：`Tracker::track()` 在 `state_ == "lost"` 时仍会求值 `target_.diverged()`；
而默认构造的 `Target` 中 EKF 状态向量为空，`diverged()` 直接访问 `ekf_.x[8]` / `x[9]`，
属于越界读（Release 下表现为随机崩溃）。这是原有代码路径，不是本次弹道改动引入的。

**修复**（`tasks/auto_aim/target.cpp`）：

```cpp
if (ekf_.x.size() < 11) return true;  // 未初始化 -> 不可用
```

修复后同一命令跑 250 帧正常退出。建议后续把 Tracker 的发散检测移入
`if (state_ != "lost")` 分支，从调用侧再收紧一次。

### 3.4 standalone 测试脚本无法编译

**现象**：`bash tests/run_outpost_usbcamera_tests.sh --standalone`

```
cc1plus: fatal error: tests/outpost_geometry_test.cpp: 没有那个文件或目录
undefined reference to `tools::solve_ballistic(...)`
```

**根因**：

- `build_standalone()` 先 `cd "$objdir"`，却仍用相对仓库根的源文件路径；
- Aimer 新接入 `tools/ballistics.cpp` 后，脚本的 `TOOLS_SRC` 没有同步加入该文件。

**修复**：

- `tests/run_outpost_usbcamera_tests.sh` 的源文件列表全部改为 `$REPO_ROOT/...` 绝对路径；
- 两个脚本的 `TOOLS_SRC` 都加入 `tools/ballistics.cpp`；
- 同步更新脚本里“OpenVINO 硬编码在 L15”的过期提示。

### 3.5 OpenVINO 路径可移植性

**现象**：`CMakeLists.txt` 和 `tasks/auto_buff/CMakeLists.txt` 把 `OpenVINO_DIR`
硬编码为 `/home/gzy/openvino_env/...`，`tasks/omniperception/CMakeLists.txt` 硬编码为
`/opt/intel/openvino_2024.6.0/...`；换机器直接配置失败。

**修复**：

- 根 `CMakeLists.txt`：保留 `-DOpenVINO_DIR=...` 优先；仅在未指定且本机 wheel 存在时回退；
  否则交给 `find_package` 搜索系统安装；
- 删除 `auto_buff` / `omniperception` 子目录里的硬编码 `set()`，统一继承根目录变量。

### 3.6 环境探测脚本误报“未找到 OpenVINO”

**现象**：`--probe-only` 明明全量构建成功，却报 `未找到 OpenVINO`。

**根因**：探测只查 `$OpenVINO_DIR`、`/opt/intel/openvino` 和 pip，未查
`/usr/lib/cmake/openvino*` 的系统包。

**修复**：增加系统 CMake 配置目录探测；现在输出
`[ OK ] OpenVINO(系统包): /usr/lib/cmake/openvino2024.6.0`。

---

## 4. 构建结果

```bash
cmake -S . -B build
cmake --build build -j6
```

- 结果：`[100%] Built target auto_buff_test`，退出码 0；
- `Built target` 统计：**48 个**；
- `build/` 根目录可执行文件：44 个；
- 关键产物：`standard`、`mt_standard`、`sentry`、`sentry_bp`、`sentry_debug`、
  `sentry_multithread`、`uav`、`uav_debug`、`mt_auto_aim_debug`、`auto_aim_test`、
  `ballistic_solver_test`、`outpost_*_test`、标定工具等；
- 编译日志：`out/logs/28-configure_final.log`、`out/logs/29-build_final.log`。

首次完整构建约 7 分钟，增量重建约 4 分钟（虚拟机 + VMware 共享目录，I/O 偏慢）。

---

## 5. 测试结果

### 5.1 单元/集成测试

| 测试 | 命令 | 结果 | 日志 |
|---|---|---|---|
| 弹道求解器 | `./build/ballistic_solver_test` | **PASS 134, FAIL 0**；端点误差 0.994 mm；100 次求解 5.37 ms | `out/logs/30-ballistic_final.log` |
| 前哨站 helper | `bash tests/run_outpost_tests.sh` | **PASS 41, FAIL 0** | `out/logs/31-outpost_final.log` |
| 前哨站几何 | 同上 | **22 passed, 0 failed** | 同上 |
| 前哨站 EKF | 同上 | **28 passed, 0 failed** | 同上 |
| 前哨站 Aimer | 同上 | **7323–7329 passed, 0 failed**（随帧时序浮动），fire_ratio≈0.33 | 同上 |
| standalone 全量 | `bash tests/run_outpost_tests.sh --standalone` | 41 / 22 / 28 / **7329 passed**，全绿 | `out/logs/35-outpost_standalone_final.log` |
| 阻力弹道接入 | `./build/outpost_aimer_test out/outpost_ballistics.yaml` | **7328–7332 passed, 0 failed**，`c=0.02010 1/m` | `out/logs/32-outpost_aimer_drag.log` |

> CMake 模式与 standalone 模式 Aimer 断言的 2 项差异来自场景 fire/preaim 边界帧计数
> （167/168 vs 168/333），两种模式均 0 失败。

### 5.2 冒烟测试（无相机，用 assets/demo 录像）

| 项目 | 命令/条件 | 结果 |
|---|---|---|
| YOLO 推理 | `detector_video_test -c=configs/demo.yaml -e=10` | 退出码 0，无崩溃 |
| 完整流水线 | `auto_aim_test -c=out/demo_lowconf.yaml -e=250` | 退出码 0，250 帧 |
| 检出统计 | `dashboard.py` 监听 UDP 9870 | 199 包，184 帧 `armor_num=1`，15 帧 0 |
| 遥测看板 | `python3 tools/debug/dashboard.py --csv ...` | 正常接收并落 CSV |

注意：`configs/demo.yaml` 的 `min_confidence: 0.8` 在该 demo 上**一帧也检不到**；
把阈值降到 `0.1`（`out/demo_lowconf.yaml`）后可稳定检出。这是模型/阈值标定问题，
不是推理链路故障；实车配置不要把阈值照抄 0.8。

### 5.3 运行时依赖

```bash
for f in build/*; do ldd "$f" 2>/dev/null | grep 'not found'; done
```

无任何 `not found`。当前二进制的 RUNPATH 包含：

```
/mnt/hgfs/RM/DIODE-27/io/hikrobot/lib/amd64
/mnt/hgfs/RM/DIODE-27/io/mindvision/lib/amd64
/home/gzy/openvino_env/lib/python3.12/site-packages/openvino/libs
```

---

## 6. 本机无法验证的部分

| 项 | 原因 | 上车前要做 |
|---|---|---|
| 外接 USB 相机实时识别 | `/dev/video0` 不存在 | 插相机后跑 `--detect --display` |
| 工业相机（Hik/MindVision） | 无设备 | 确认 SDK 库、曝光、`vid_pid` |
| CAN 通信 | 无 `can0`、无 can-utils | `ip link set can0 up type can bitrate 1000000` |
| 下位机串口 | 无 `/dev/ttyACM*`、`/dev/ttyUSB*` | 配 udev 软链 `/dev/gimbal`，确认 9600 8N1 |
| 云台闭环/开火 | 无下位机 | 跑 `gimbal_test`、`fire_test`、`cboard_test` |
| 阻力系数 `c` 标定 | 需靶纸/测速 | 按 `docs/ballistics_design.md` §5 落点法标定 |
| 标定程序运行 | 无标定图片/棋盘格 | ChArUco 兼容性只验证了编译，未跑数据 |
| 推流/录制/Planner 参考 | 功能未实现（计划 B1–B3/C3） | 见 `docs/plan_master.md` |

---

## 7. 复现命令

```bash
cd /mnt/hgfs/RM/DIODE-27

# 1) 配置 + 全量构建
cmake -S . -B build -DOpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0   # 可选，指定系统 OpenVINO
cmake --build build -j"$(nproc)"

# 2) 离线单测
./build/ballistic_solver_test
bash tests/run_outpost_tests.sh
bash tests/run_outpost_tests.sh --standalone

# 3) 环境探测（无设备时应看到 camera/CAN/serial 的 WARN，而不是 FAIL）
bash tests/run_outpost_usbcamera_tests.sh --probe-only

# 4) 无相机冒烟（demo 阈值需调低，见 §5.2）
env QT_QPA_PLATFORM=offscreen ./build/auto_aim_test -c=out/demo_lowconf.yaml -e=100 assets/demo/demo

# 5) 启用阻力弹道的 Aimer 集成验证
./build/outpost_aimer_test out/outpost_ballistics.yaml
```

---

## 8. 残余风险与建议

1. **OpenVINO RUNPATH 指向 `/home/gzy/openvino_env`**：换机器/上机器人会找不到
   `libopenvino.so.2610`。部署时用系统 OpenVINO 重新构建，或设置 `LD_LIBRARY_PATH`。
2. **`Target::diverged()` 的越界保护是防御性修复**：建议把 Tracker 的发散/门控检查
   移到 `state_ != "lost"` 分支内，避免对未初始化目标做任何求值。
3. **阻力弹道默认关闭**，且 `c=0.0201` 是物理估算值，不是实弹标定值。直接启用会系统性偏低，
   按 `docs/ballistics_design.md` 标定后再上车。
4. **demo 的 `min_confidence=0.8` 不适配当前模型**：实车配置需用现场录像重新标定阈值。
5. **标定程序只做了编译兼容**：OpenCV 4.6 分支的 ChArUco 逻辑未用真实图片跑过，首次标定需复核角点顺序。
6. **未接入 CTest**：测试仍靠脚本驱动；建议后续 `enable_testing()` + `add_test()`，
   把 4 个前哨站测试和弹道测试纳入 `ctest`。

---

## 9. 本次修改文件清单

| 文件 | 修改 |
|---|---|
| `CMakeLists.txt` | 删除重复 `ballistic_solver_test`；OpenVINO 路径改为可覆盖/可回退 |
| `tasks/auto_aim/CMakeLists.txt` | 删除硬编码 `OpenVINO_DIR` |
| `tasks/auto_buff/CMakeLists.txt` | 删除硬编码 `OpenVINO_DIR` |
| `tasks/omniperception/CMakeLists.txt` | 删除硬编码 `OpenVINO_DIR` |
| `calibration/calibrate_camera.cpp` | OpenCV 4.6/4.7+ 与 yaml-cpp 0.8 兼容 |
| `tasks/auto_aim/target.cpp` | 未初始化 EKF 的越界保护 |
| `tests/run_outpost_tests.sh` | standalone 增加 `ballistics.cpp`；更新错误提示 |
| `tests/run_outpost_usbcamera_tests.sh` | standalone 改绝对路径；增加 `ballistics.cpp`；修正 OpenVINO 探测 |
| `docs/verification_report.md` | 本报告 |
