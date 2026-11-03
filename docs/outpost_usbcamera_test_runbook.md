# 外接相机 · 前哨站自瞄测试指令手册（Ubuntu 22.04+）

> 面向「外接相机（USB / UVC）」的机器人：照着本文从上往下敲命令，即可把**前哨站自瞄**跑起来并做测试。
> 逐条命令都可直接复制；每条命令后面标注了**怎么判断成功**与**失败先看哪里**。
>
> 配套文件：
> - 专用配置 [configs/outpost_test.yaml](../configs/outpost_test.yaml)
> - 一键脚本 [tests/run_outpost_usbcamera_tests.sh](../tests/run_outpost_usbcamera_tests.sh)
> - 流程与代码位置 [docs/outpost_flow_summary.md](outpost_flow_summary.md)
> - 标定流程 [ChArUco_Calibration_Guide.md](../ChArUco_Calibration_Guide.md)

---

## 0. 先明确：前哨站自瞄没有“开关”

前哨站链路随识别结果**自动生效**：

- 识别器给出 `armor.name == outpost` → `Tracker::set_target` 自动构造 `OutpostTarget`（5 维 EKF + 6 点 PnP 测距）；
- 其他兵种照旧走 11 维 `Target`，两条路径互不干扰。

所以“开启自瞄打前哨站”实际只需要三件事同时成立：

| # | 条件 | 检查方式 |
|---|---|---|
| 1 | 程序在跑，且相机在出帧 | 终端出现 `USBCamera opened` / `fps` 日志 |
| 2 | `enemy_color` 与场上前哨站颜色一致，且配置有 `outpost_max_temp_lost_count` | §3 配置检查 |
| 3 | 下位机云台处于自瞄档 | 终端出现 `[CBoard] ... Mode: auto_aim` 或 `Switch to auto_aim` |

第 3 条两种通信方式不同：

- **CAN（`io::CBoard`）**：下位机下发 `Mode`，`auto_aim(1)` 与 `outpost(4)` 在无人机里等价；步兵 `standard` 无档位过滤。
- **串口（`io::Gimbal`）**：上位机从串口帧里读 `mode`，下位机给 `1` 即自瞄。

### 0.1 识别画面的来源：相机直连视觉计算单元，**不经图传**

本仓库的识别图像只有三种来源，都**不是**图传（VTX / 无线链路）：

| 来源 | 代码位置 | 连接方式 |
|---|---|---|
| `io::Camera` → `HikRobot` | [io/camera.cpp:23](../io/camera.cpp#L23)、[io/hikrobot/hikrobot.cpp:62](../io/hikrobot/hikrobot.cpp#L62) | 海康 USB3 工业相机，`MV_CC_EnumDevices(MV_USB_DEVICE)`，厂商 SDK 直连 |
| `io::Camera` → `MindVision` | [io/camera.cpp:17](../io/camera.cpp#L17)、[io/mindvision/mindvision.cpp:67](../io/mindvision/mindvision.cpp#L67) | 迈德威视 USB 工业相机，`CameraSdkInit` + `CameraEnumerateDevice` 直连 |
| `io::USBCamera` | [io/usbcamera/usbcamera.cpp:96](../io/usbcamera/usbcamera.cpp#L96) | `cv::VideoCapture("/dev/videoN", CAP_V4L)`，UVC 直连 |

另外只有离线文件：`auto_aim_test` / `detector_video_test` 读本地 `.avi`，标定读 `cv::imread`。
**没有** RTSP / GStreamer / UDP 流输入，也没有 ROS 图像话题（`io/ros2` 已于 2026-11 去 ROS 化时删除，见 [auto_aim_upgrade_plan.md](auto_aim_upgrade_plan.md)）。

图传在比赛里是**另一条链路**（给操作手/裁判看画面），是压缩+有延迟的，不能用于自瞄测量。所以标准接法就是：
相机 USB 直插视觉计算单元（mini PC / Jetson），参数由 `vid_pid`（工业相机）或 `/dev/videoN`（UVC）指定。

> **如果你的硬件是「相机 → 图传发射 → 图传接收 → USB 采集卡 → /dev/videoN」**，代码照样能跑（被当成 `USBCamera`），
> 但以下四点会直接损害前哨站精度，需要先评估：
> 1. **延迟未补偿**：三处取图的时间戳都是**帧到达时刻** `steady_clock::now()`
>    （[hikrobot.cpp:118](../io/hikrobot/hikrobot.cpp#L118)、[mindvision.cpp:100](../io/mindvision/mindvision.cpp#L100)、[usbcamera.cpp:149](../io/usbcamera/usbcamera.cpp#L149)），
>    没有用相机硬件时间戳；而 [standard.cpp:59](../src/standard.cpp#L59) 用同一时刻的 IMU 四元数解算。
>    图传延迟（常见 50–150 ms）会变成系统性 yaw 偏差（云台 2 rad/s 时约 6–17°），前哨站板只有 135 mm 宽，基本打不上。
> 2. **压缩伪影**：图传多为 H.264，灯条边缘糊，传统识别阈值与 PnP 测距都会退化。
> 3. **掉帧/抖动**：[tracker.cpp:43-46](../tasks/auto_aim/tracker.cpp#L43-L46) 在 `dt > 0.1 s` 时直接置 `lost`。
> 4. **帧率不匹配**：代码按 60–150 fps 设计（[hikrobot.cpp:90](../io/hikrobot/hikrobot.cpp#L90) 设 150 fps），图传通常 30–60 fps 且延迟抖动。
>
> 自查方法：手机秒表对准相机，同时截屏电脑上的识别画面，两者读数差即为端到端延迟；超过 1 帧（60 fps 时 16.7 ms）就要在图传链路上找补或在 `Aimer` 里加固定延迟补偿。

---

## 1. 场景 A：只验证算法（不用相机、不用下位机，5 分钟）

这是**最先该跑**的一步，能在没有硬件的情况下确认前哨站链路本身没问题。

```bash
cd ~/DIODE-27                       # 换成你的仓库路径
sudo apt update
sudo apt install -y build-essential cmake pkg-config \
                    libeigen3-dev libopencv-dev libyaml-cpp-dev \
                    libspdlog-dev libfmt-dev nlohmann-json3-dev \
                    v4l-utils can-utils iproute2

# 一键：环境自检 + 编译 + 4 个前哨站测试（不需要 OpenVINO / 相机 SDK）
bash tests/run_outpost_usbcamera_tests.sh --standalone
```

**期望结果**：4 个测试全 PASS，退出码 0

| 测试 | 覆盖内容 | 参考通过数 |
|---|---|---|
| `outpost_helper_test` | HeightBuffer / 切板方向确认 / 四象限高度差 / 折算 / 门控 | PASS 41 / FAIL 0 |
| `outpost_geometry_test` | 邻侧板几何推算 / 灯条端点 / 板序号 | 22 passed / 0 failed |
| `outpost_target_test` | 5 维 EKF 收敛（中心 <0.05 m、ω <0.1 rad/s） | 28 passed / 0 failed |
| `outpost_aimer_test` | 快照三板高度 / 击攻窗口 / 滞回 / 预瞄 / ω 退化 | 约 7327 passed / 0 failed |

日志落在 `build/outpost_usbcamera_logs/*.log`。失败先看对应 `build_*.log`；CMake 报 OpenVINO 找不到时，**这正是用 `--standalone` 的原因**（仓库把 `OpenVINO_DIR` 硬编码在 `CMakeLists.txt:15` 与 `tasks/auto_aim/CMakeLists.txt:3`）。

只看环境与设备、不编译：

```bash
bash tests/run_outpost_usbcamera_tests.sh --probe-only
```

---

## 2. 场景 B：外接相机 + 整机自瞄（比赛/联调用）

### 2.1 依赖与 OpenVINO

```bash
# 先看 OpenVINO 是否可用（YOLO 推理必需）
python3 -c "import openvino,os;print(os.path.join(os.path.dirname(openvino.__file__),'cmake'))"

# 情况 1：输出一个存在的目录 → 直接把它传给 cmake（推荐，不改源码）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOpenVINO_DIR=<上一条命令输出的路径>

# 情况 2：完全没装 OpenVINO，且只想验证前哨站算法
bash tests/run_outpost_usbcamera_tests.sh --standalone
```

若想让仓库默认就能配置，可把两条硬编码路径改成自己的：

```bash
sed -i 's#^set(OpenVINO_DIR .*#set(OpenVINO_DIR "'"$HOME"'/openvino_env/lib/python3.12/site-packages/openvino/cmake")#' CMakeLists.txt
sed -i 's#^set(OpenVINO_DIR .*#set(OpenVINO_DIR "'"$HOME"'/openvino_env/lib/python3.12/site-packages/openvino/cmake")#' tasks/auto_aim/CMakeLists.txt
grep -n "OpenVINO_DIR" CMakeLists.txt tasks/auto_aim/CMakeLists.txt
```

### 2.2 外接相机识别与权限

```bash
# 找到相机节点（插拔前后各跑一次对比最稳）
v4l2-ctl --list-devices
ls -l /dev/video*

# 看支持的分辨率/帧率/格式
v4l2-ctl -d /dev/video0 --list-formats-ext

# 看/改曝光（前哨站灯条不能糊，曝光别太大）
v4l2-ctl -d /dev/video0 --get-ctrl=exposure_time_absolute,exposure_auto,gain

# 权限：临时（当前会话有效）
sudo chmod a+rw /dev/video0

# 权限：永久（推荐）
sudo usermod -aG video "$USER"
# 重新登录（或 newgrp video）后生效：
groups | tr ' ' '\n' | grep -x video

# 快速验证相机能出图（OpenCV 直接读）
python3 -c "
import cv2;c=cv2.VideoCapture('/dev/video0');print('opened',c.isOpened());
ok,f=c.read();print('frame',ok, None if f is None else f.shape);c.release()"
```

要点（与代码实现对应）：

- `io::USBCamera` 内部打开的是 `"/dev/" + name`，命令行 `-n video0` 就对应 `/dev/video0`；`-n video2` 对应 `/dev/video2`。
- 它强制设置 `CAP_PROP_FOURCC = MJPG`，所以相机**必须支持 MJPG**，否则会掉帧或打不开。
- 它读取的配置键是 `image_width / image_height / usb_exposure / usb_frame_rate / usb_gamma / usb_gain`——注意**不是** `camera_name / vid_pid / exposure_ms` 那一组（那组是工业相机 `io::Camera` 用的）。
- 相机掉线会被守护线程反复重连，超过 20 次才放弃并在日志里打印 `Give up to open ... USB camera`。

### 2.3 下位机通信口

**CAN：**

```bash
sudo modprobe gs_usb                                 # USB-CAN 常见驱动
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
ip -details link show can0                           # state UP 才算好

candump can0                                         # 看到 0x100(四元数)/0x110(弹速+档位) 即有数据
cansend can0 0ff#0000000000000000                     # 手动发一帧空指令自测
```

`Mode: auto_aim` 出现即下位机已切自瞄档（`[CBoard] Bullet speed: ..., Mode: ..., Shoot mode: ...`）。

**串口（`com_port`）：**

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
sudo chmod a+rw /dev/ttyACM0

# 推荐：固定设备名（避免 ttyUSB 号漂移）
sudo tee /etc/udev/rules.d/99-rm-gimbal.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="gimbal", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/gimbal            # 出现即 OK；不出现把 idVendor/idProduct 换成 lsusb 里你自己的值
```

注意 `io::Gimbal` 用的是串口库默认波特率（9600）。若下位机不是 9600，需要改 `io/gimbal/gimbal.cpp` 里 `serial_.setBaudrate(...)`。

### 2.4 配置准备（前哨站专项）

```bash
cp configs/outpost_test.yaml configs/outpost_$(date +%m%d).yaml   # 建议先备份一份
$EDITOR configs/outpost_test.yaml
```

必须改的键：

| 键 | 取值 | 说明 |
|---|---|---|
| `enemy_color` | `"red"` / `"blue"` | 必须等于场上前哨站颜色，错了会“看得见但永不打” |
| `outpost_max_temp_lost_count` | `75`（无人机 `150`） | **必需键**，缺失时 `Tracker` 构造直接抛 YAML 异常 |
| `camera_matrix` / `distort_coeffs` | 你标定的值 | 外接相机与工业相机内参不同，**必须重标定**，否则测距/瞄准系统性偏 |
| `R_camera2gimbal` / `t_camera2gimbal` | 你标定的值 | 手眼外参，同上 |
| `image_width/height`、`usb_*` | 按实际相机 | 用 `v4l2-ctl --list-formats-ext` 的结果对齐 |

前哨站相关可调键（都有缺省值，`aimer.cpp:32-46` 读取）：

| 键 | 缺省 | 作用 |
|---|---|---|
| `attack_window` | 40° | 攻击窗口全幅，与动态窗口取 min |
| `window_hysteresis` | 0.2 | 施密特滞回：enter=half×0.8 / exit=half×1.2 |
| `degraded_angle_speed` | 12 rad/s | 超过则不输出云台控制（前哨站 0.8π≈2.51 不会触发） |
| `window_redundancy` | 0.8 | 云台速度/加速度裕度 |
| `max_yaw_vel` / `max_yaw_acc` | 10 / 200 | 动态窗口反解用，按实车云台填 |
| `attack_preaim` | false | 边界预瞄点是否允许开火（false=只跟不打） |
| `auto_fire` | true | 首次联调建议先设 `false`，只出瞄准量不开火 |

几何常量（半径 0.2765 m、板高差 0.102 m、板 pitch −15°、切板阈值 50°、方向确认 2 次）目前**编译期固定**在 `tasks/auto_aim/outpost_const.hpp`，改完要重新编译。

配置自检（脚本会自动做同样的检查）：

```bash
for k in outpost_max_temp_lost_count enemy_color camera_matrix distort_coeffs \
         R_camera2gimbal t_camera2gimbal image_width usb_frame_rate usb_exposure; do
  grep -qE "^[[:space:]]*$k:" configs/outpost_test.yaml && echo "OK   $k" || echo "MISS $k"
done
```

> 顺带修了一个坑：[configs/example.yaml](../configs/example.yaml) 原本缺 `outpost_max_temp_lost_count`，用它启动会直接抛异常，现已补上。

### 2.5 编译

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"

# 只想要前哨站测试目标（不碰 OpenVINO）：
cmake --build build --target outpost_helper_test outpost_geometry_test \
                          outpost_target_test outpost_aimer_test -j"$(nproc)"
```

### 2.6 先跑：外接相机实时识别（不下发、不动云台）

```bash
# 有桌面的机器：弹窗看识别框，按 q 退出
./build/usbcamera_detect_test -n video0 -d configs/outpost_test.yaml

# 无桌面的机器（SSH）：程序本身不会自己退出，用 timeout 限时跑，然后看日志
timeout 15s ./build/usbcamera_detect_test -n video0 configs/outpost_test.yaml
# 一键脚本里已封装这一步：bash tests/run_outpost_usbcamera_tests.sh --detect
```

**判断成功**：出现 `USBCamera opened`、`fps` 稳定在你配置的帧率附近；窗口里前哨站三块板应出现检测框。
前哨站**只能是小装甲板**（135 mm 宽、灯条 56 mm），识别器里 `check_type/get_type` 对大板假设会直接排除 outpost。

### 2.7 再跑：整链路自瞄（会真实控制云台）

```bash
# 无人机（有独立 outpost(4) 档，与 auto_aim(1) 等价）
./build/uav configs/outpost_test.yaml

# 步兵 / 通用（识别到前哨站自动进 5 维链路）
./build/standard configs/outpost_test.yaml

# 多线程 + 绘图版（帧率更高，方便看 NIS/协方差）
./build/mt_auto_aim_debug configs/outpost_test.yaml

# 停止：Ctrl+C（tools::Exiter 会优雅退出）
```

上机前建议的“降风险三步”：

1. 配置里 `auto_fire: false` → 只出瞄准量、不开火；
2. `attack_preaim: false`（默认）→ 窗口内无板时只跟踪不打；
3. 先让云台空转看跟踪是否稳定，再打开 `auto_fire`。

### 2.8 一键跑完 B 场景

```bash
# 环境自检 + 编译 + 单测 + 相机出帧检查，并生成闭环命令清单
bash tests/run_outpost_usbcamera_tests.sh --detect

# 加上弹窗显示
bash tests/run_outpost_usbcamera_tests.sh --detect --display

# 顺带跑一次录像回放整链路
bash tests/run_outpost_usbcamera_tests.sh --video assets/demo/demo
```

---

## 3. 场景 C：录像回放（可重复、最推荐做回归）

```bash
# 录像 + 配套 IMU 四元数序列：<前缀>.avi + <前缀>.txt（每行 t w x y z）
./build/auto_aim_test -c configs/outpost_test.yaml -s 0 -e 0 assets/demo/demo

# 无桌面环境（Plotter 要建窗口）
xvfb-run -a ./build/auto_aim_test -c configs/outpost_test.yaml assets/demo/demo

# 只看识别（不跑 Tracker/Aimer）
./build/detector_video_test -c configs/outpost_test.yaml assets/demo/demo.avi
```

自己采一段前哨站录像（用机器人的 `tools::Recorder`，或先用 ffmpeg 凑合看识别）：

```bash
ffmpeg -f v4l2 -input_format mjpeg -framerate 60 -video_size 1920x1080 \
       -i /dev/video0 -t 20 -c:v copy assets/outpost_raw.mp4
```

前哨站回归要点：录像里三块板循环切换，检查是否**只喂最匹配的一块板**、切板方向锁定后无反向误判、`abs_ref_index` 与真值一致。

---

## 4. 场景 D：单点排查命令（按现象查）

| 现象 | 先跑这条 | 期望 / 处理 |
|---|---|---|
| 程序起不来，报 YAML 异常 | `grep -n outpost_max_temp_lost_count configs/*.yaml` | 缺这个键的配置都会抛异常，补上 |
| 相机打不开 | `ls -l /dev/video0 && v4l2-ctl -d /dev/video0 --list-formats-ext` | 无 MJPG 或权限不足；`sudo usermod -aG video $USER` |
| 能识别但从不打前哨站 | `grep -n enemy_color configs/outpost_test.yaml` | 颜色反了；或 `auto_fire: false`；或 `attack_preaim` 相关 |
| 测距/瞄准系统性偏 | 重标定内参与手眼，见 `ChArUco_Calibration_Guide.md` | 外接相机必须重标定 |
| 云台不动 | `candump can0` 或串口收包 | CAN 无 0x100/0x110 → 下位机没发；档位不是 auto_aim |
| 跟踪丢失/发散 | `grep -E "diverged|Bad Converge|Large dt" 日志` | 对应 `[Tracker] Target diverged!` / NIS 差 / 相机离线 |
| ω 相关异常 | 看日志 `[Aimer] Outpost attack window degraded: omega=` | |ω|>12 rad/s 会退化不输出控制 |

云端/无桌面机器上跑带窗口的程序，统一前面加 `xvfb-run -a`。

---

## 5. 前哨站测试命令速查表

```bash
# 算法层（无需硬件）
bash tests/run_outpost_usbcamera_tests.sh --standalone
./build/outpost_helper_test
./build/outpost_geometry_test
./build/outpost_target_test
./build/outpost_aimer_test configs/outpost_test.yaml

# 外接相机
./build/usbcamera_test -n video0 -d configs/outpost_test.yaml          # 只看相机
./build/usbcamera_detect_test -n video0 -d configs/outpost_test.yaml   # 相机 + 识别
./build/multi_usbcamera_test -d configs/outpost_test.yaml              # 多相机（video0/video2 + 工业相机）

# 整链路
./build/auto_aim_test -c configs/outpost_test.yaml assets/demo/demo    # 录像回放
./build/uav configs/outpost_test.yaml                                  # 无人机闭环
./build/standard configs/outpost_test.yaml                             # 步兵闭环
./build/mt_auto_aim_debug configs/outpost_test.yaml                    # 多线程调试

# 标定
./build/capture configs/calibration.yaml -o assets/calib
./build/calibrate_camera configs/calibration.yaml
./build/calibrate_handeye configs/calibration.yaml
```

---

## 6. 关键日志关键字

| 关键字 | 含义 | 位置 |
|---|---|---|
| `USBCamera opened` / `[left USB camera] capture thread started` | 外接相机已打开并取流 | `io/usbcamera/usbcamera.cpp` |
| `Give up to open ... USB camera` | 重连 20 次仍失败 | 同上 |
| `Switch to auto_aim` / `Switch to outpost` | 下位机档位切换到自瞄 | `src/uav.cpp`、`src/standard.cpp` |
| `[Tracker] Target diverged!` | EKF 发散（协方差>150 / 位置>15 m / ω>10π / 500 ms 未更新） | `tasks/auto_aim/tracker.cpp` |
| `[Target] Bad Converge Found!` | NIS 失败比例过高 | 同上 |
| `[Aimer] Outpost attack window degraded: omega=` | ω 过大，退化不输出控制 | `tasks/auto_aim/aimer.cpp` |
| `[Tracker] Large dt` | 相机离线（dt>0.1 s） | 同上 |

调试可视化（这三处都会画 `aimer.debug_aim_point`，是看攻击窗口/滞回/预瞄最直接的入口）：

```bash
./build/mt_auto_aim_debug configs/outpost_test.yaml
./build/uav_debug        configs/outpost_test.yaml
./build/sentry_debug     configs/sentry.yaml     # 注意：哨兵 Decider 会过滤前哨站，规则上不打
```

绿色点 = 预测三板位姿，红点 = 当前瞄准点（开火），蓝点 = 预瞄点（只跟踪不开火）。

---

## 7. 已知限制（测试时需要知道）

1. **只支持小装甲板**：outpost 被强制为 small（135 mm 宽、灯条 56 mm）。
2. **哨兵不打前哨站**：`tasks/omniperception/decider.cpp` 会 remove 掉前哨站，`sentry.yaml` 里的前哨站参数仅为不报错。
3. **6 点 PnP 的 2 个额外点是几何推算的**：`tracker.cpp` 传 `{-1,-1}` 占位，`solver.cpp` 把推算出的世界系端点投回图像，目前是“纯几何闭环”，未使用检测器真实检出的邻侧灯条。
4. **半径沿用旧值 0.2765 m**（rmcs 为 0.275，差 1.5 mm），未重新标定。
5. **几何常量未暴露到 YAML**，改需重新编译。
6. **快照 ω 钳位**：`Target::predict` 把收敛后的前哨站 |ω|>2 钳到 2.51 rad/s，故快照外推转速比 5 维 EKF 原始值低约 0.3%。

---

## 8. 一页命令流（最小可用）

```bash
cd ~/DIODE-27
sudo apt install -y build-essential cmake pkg-config libeigen3-dev libopencv-dev \
                    libyaml-cpp-dev libspdlog-dev libfmt-dev nlohmann-json3-dev v4l-utils
bash tests/run_outpost_usbcamera_tests.sh --standalone            # 1) 算法层全绿
python3 -c "import openvino,os;print(os.path.join(os.path.dirname(openvino.__file__),'cmake'))"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOpenVINO_DIR=<上面的路径>
cmake --build build -j"$(nproc)"                                  # 2) 全量编译
v4l2-ctl --list-devices && sudo chmod a+rw /dev/video0            # 3) 相机
./build/usbcamera_detect_test -n video0 -d configs/outpost_test.yaml   # 4) 识别
./build/uav configs/outpost_test.yaml                             # 5) 闭环自瞄
```
