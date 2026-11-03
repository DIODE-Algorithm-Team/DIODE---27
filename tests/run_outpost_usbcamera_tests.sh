#!/usr/bin/env bash
# =============================================================================
# 前哨站（Outpost）外接相机测试一键脚本  ——  Ubuntu 22.04+
#
# 只做四件事，任一环节失败都会明确告诉你原因：
#   [1/5] 环境自检       依赖 / 头文件 / 相机与通信设备
#   [2/5] 编译           前哨站 4 个测试目标（可用 --standalone 绕开 OpenVINO）
#   [3/5] 离线链路测试   EKF / 切板 / 板序号 / 快照 / Aimer 窗口与开火
#   [4/5] 实机链路测试   外接相机识别 →（可选）录像回放 →（可选）闭环自瞄
#   [5/5] 结果汇总       日志落盘，退出码：0=全通过 1=有失败 2=环境不满足
#
# 用法：
#   bash tests/run_outpost_usbcamera_tests.sh                 # 默认：环境自检 + 单测 + 相机探测
#   bash tests/run_outpost_usbcamera_tests.sh --standalone     # 不需要 OpenVINO / 相机 SDK
#   bash tests/run_outpost_usbcamera_tests.sh --probe-only     # 只看环境和设备，不编译
#   bash tests/run_outpost_usbcamera_tests.sh --video assets/demo/demo   # 追加录像回放整链路
#   bash tests/run_outpost_usbcamera_tests.sh --detect --display         # 外接相机实时识别（弹窗）
#   bash tests/run_outpost_usbcamera_tests.sh --config configs/outpost_test.yaml
#   bash tests/run_outpost_usbcamera_tests.sh --video-device video0 --width 1920 --height 1080
#   bash tests/run_outpost_usbcamera_tests.sh --clean --regression
#
# 说明：
#   * 前哨站链路没有独立开关——识别结果 name==outpost 就自动切 5 维链路，
#     所以“开启自瞄打前哨站”= 跑起来 + 敌我颜色配对 + 下位机切到自瞄档。
#   * USB 外接相机走 io::USBCamera，需要配置里有 image_width/image_height/usb_* 键。
# =============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT" || exit 2

# ------------------------------ 参数与默认值 ------------------------------
MODE="cmake"                 # cmake | standalone
BUILD_DIR="$REPO_ROOT/build"
CONFIG="configs/outpost_test.yaml"
VIDEO_DEVICE="video0"
VIDEO_WIDTH="1920"
VIDEO_HEIGHT="1080"
VIDEO_PATH=""
WITH_REGRESSION=0
WITH_COVERAGE=0
WITH_DETECT=0
WITH_DISPLAY=0
DO_CLEAN=0
PROBE_ONLY=0
JOBS="$(nproc 2>/dev/null || echo 4)"

TARGETS=(outpost_helper_test outpost_geometry_test outpost_target_test outpost_aimer_test)

usage() { sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --standalone)   MODE="standalone" ;;
    --cmake)        MODE="cmake" ;;
    --coverage)     WITH_COVERAGE=1 ;;
    --clean)        DO_CLEAN=1 ;;
    --regression)   WITH_REGRESSION=1 ;;
    --detect)       WITH_DETECT=1 ;;
    --display)      WITH_DISPLAY=1 ;;
    --probe-only)   PROBE_ONLY=1 ;;
    --video)        VIDEO_PATH="${2:-}"; shift ;;
    --config)       CONFIG="${2:-}"; shift ;;
    --build-dir)    BUILD_DIR="${2:-}"; shift ;;
    --video-device) VIDEO_DEVICE="${2:-}"; shift ;;
    --width)        VIDEO_WIDTH="${2:-}"; shift ;;
    --height)       VIDEO_HEIGHT="${2:-}"; shift ;;
    -j|--jobs)      JOBS="${2:-}"; shift ;;
    -h|--help)      usage; exit 0 ;;
    *) echo "[ERROR] 未知参数: $1"; usage; exit 2 ;;
  esac
  shift
done

LOG_DIR="$BUILD_DIR/outpost_usbcamera_logs"
BIN_DIR="$LOG_DIR/bin"
COV_DIR="$LOG_DIR/cov"
mkdir -p "$LOG_DIR" "$BIN_DIR"

if [ -t 1 ]; then
  C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'; C_CYN=$'\033[36m'; C_OFF=$'\033[0m'
else
  C_RED=""; C_GRN=""; C_YEL=""; C_CYN=""; C_OFF=""
fi
say()  { echo "${C_CYN}$*${C_OFF}"; }
ok()   { echo "  ${C_GRN}[ OK ]${C_OFF} $*"; }
warn() { echo "  ${C_YEL}[WARN]${C_OFF} $*"; }
bad()  { echo "  ${C_RED}[FAIL]${C_OFF} $*"; }

R_NAME=(); R_STATUS=(); R_DETAIL=()
record() { R_NAME+=("$1"); R_STATUS+=("$2"); R_DETAIL+=("$3"); }
need()   { command -v "$1" >/dev/null 2>&1; }

echo "=============================================================="
echo " 前哨站外接相机测试"
echo "   仓库根目录 : $REPO_ROOT"
echo "   配置       : $CONFIG"
echo "   构建目录   : $BUILD_DIR"
echo "   构建模式   : $MODE"
echo "   日志目录   : $LOG_DIR"
echo "=============================================================="

# =============================================================================
# [1/5] 环境自检
# =============================================================================
say ""
say "[1/5] 环境自检"

ENV_FAIL=0
check_cmd() {
  local c="$1" hint="$2" optional="${3:-0}"
  if need "$c"; then ok "$c: $(command -v "$c")"
  elif [ "$optional" = "1" ]; then warn "$c 未安装（可选）  $hint"
  else bad "$c 未安装  $hint"; ENV_FAIL=1; fi
}
check_cmd bash        "本脚本需要 bash" 0
check_cmd cmake       "sudo apt install -y cmake" 0
check_cmd g++         "sudo apt install -y build-essential" 0
check_cmd make        "sudo apt install -y build-essential" 0
check_cmd pkg-config  "sudo apt install -y pkg-config" 0
check_cmd v4l2-ctl    "sudo apt install -y v4l-utils（用于相机格式/曝光排查）" 1
check_cmd gcovr       "pip install gcovr（--coverage 才需要）" 1
check_cmd ip          "sudo apt install -y iproute2" 0
check_cmd can-utils   "sudo apt install -y can-utils（CAN 排查用）" 1

# 编译期头文件
for hdr in /usr/include/eigen3/Eigen/Dense \
           /usr/include/yaml-cpp/yaml.h \
           /usr/include/spdlog/spdlog.h \
           /usr/include/fmt/core.h \
           /usr/include/nlohmann/json.hpp; do
  if [ -f "$hdr" ]; then ok "头文件 $hdr"
  else bad "缺少 $hdr  →  sudo apt install -y libeigen3-dev libyaml-cpp-dev libspdlog-dev libfmt-dev nlohmann-json3-dev"; ENV_FAIL=1; fi
done
if pkg-config --exists opencv4; then ok "OpenCV $(pkg-config --modversion opencv4)"
else bad "缺少 OpenCV  →  sudo apt install -y libopencv-dev"; ENV_FAIL=1; fi

# OpenVINO（只有 cmake 全量构建 / YOLO 推理才需要）
OVINO_FOUND=0
OVINO_DIR_HINT=""
if [ -n "${OpenVINO_DIR:-}" ] && [ -d "${OpenVINO_DIR}" ]; then
  OVINO_FOUND=1; OVINO_DIR_HINT="$OpenVINO_DIR"; ok "OpenVINO_DIR=$OpenVINO_DIR"
elif [ -d /opt/intel/openvino/runtime/cmake ]; then
  OVINO_FOUND=1; OVINO_DIR_HINT="/opt/intel/openvino/runtime/cmake"; ok "OpenVINO: /opt/intel/openvino"
elif [ -n "$(ls -d /usr/lib/cmake/openvino* /usr/local/lib/cmake/openvino* 2>/dev/null | head -n 1)" ]; then
  OVINO_DIR_HINT="$(ls -d /usr/lib/cmake/openvino* /usr/local/lib/cmake/openvino* 2>/dev/null | head -n 1)"
  OVINO_FOUND=1; ok "OpenVINO(系统包): $OVINO_DIR_HINT"
elif python3 -c 'import openvino,os,sys; sys.stdout.write(os.path.join(os.path.dirname(openvino.__file__),"cmake"))' >/tmp/.ov_dir 2>/dev/null; then
  OVINO_DIR_HINT="$(cat /tmp/.ov_dir)"
  [ -d "$OVINO_DIR_HINT" ] && { OVINO_FOUND=1; ok "OpenVINO(pip): $OVINO_DIR_HINT"; }
fi
if [ "$OVINO_FOUND" = "0" ]; then
  warn "未找到 OpenVINO：cmake 全量构建会失败"
  warn "  → 只验证前哨站算法：bash tests/run_outpost_usbcamera_tests.sh --standalone"
  warn "  → 要做实机识别：安装 OpenVINO（apt / pip），或用 -DOpenVINO_DIR=... 指定路径"
fi

# 配置文件必需键
if [ -f "$CONFIG" ]; then
  ok "配置文件存在：$CONFIG"
  for key in outpost_max_temp_lost_count enemy_color min_detect_count max_temp_lost_count \
             camera_matrix distort_coeffs R_camera2gimbal t_camera2gimbal; do
    if grep -qE "^[[:space:]]*${key}:" "$CONFIG"; then ok "  必需键 $key"
    else bad "  配置缺少必需键 $key（Tracker/Solver 构造会抛异常）"; ENV_FAIL=1; fi
  done
  if grep -qE "^[[:space:]]*usb_frame_rate:" "$CONFIG"; then
    for key in image_width image_height usb_exposure usb_gamma usb_gain; do
      grep -qE "^[[:space:]]*${key}:" "$CONFIG" && ok "  USB 相机键 $key" \
        || warn "  配置缺少 $key：io::USBCamera 构造会抛异常（只有用外接相机才需要）"
    done
  else
    warn "配置里没有 usb_* 键 → 该配置只适合工业相机；外接相机请用 configs/outpost_test.yaml"
  fi
else
  bad "配置文件不存在：$CONFIG"
  ENV_FAIL=1
fi

# 设备
say ""
say "  —— 设备探测（外接相机 / 通信口）——"
V4L_DEV="/dev/$VIDEO_DEVICE"
if [ -e "$V4L_DEV" ]; then
  ok "$V4L_DEV 存在"
  if need v4l2-ctl; then
    v4l2-ctl -d "$V4L_DEV" --list-formats-ext 2>/dev/null | head -n 25 | sed 's/^/       /' \
      >> "$LOG_DIR/v4l2_formats.txt"
    ok "   支持格式已写入 $LOG_DIR/v4l2_formats.txt"
    if v4l2-ctl -d "$V4L_DEV" --list-formats-ext 2>/dev/null | grep -q "MJPG"; then
      ok "   支持 MJPG（USBCamera 会强制 MJPG，缺这条会打不开或掉帧）"
    else
      warn "   未见 MJPG，USBCamera 设的四字节码可能不生效"
    fi
    EXP="$(v4l2-ctl -d "$V4L_DEV" --get-ctrl=exposure_time_absolute 2>/dev/null || true)"
    [ -n "$EXP" ] && ok "   当前曝光 $EXP"
  else
    warn "未装 v4l-utils，跳过格式检查（sudo apt install -y v4l-utils）"
  fi
  if [ -w "$V4L_DEV" ]; then ok "   当前用户可读写 $V4L_DEV"
  else warn "   当前用户对 $V4L_DEV 无写权限 → sudo usermod -aG video \$USER && 重新登录"; fi
else
  warn "$V4L_DEV 不存在（没插外接相机 / 设备号不是 $VIDEO_DEVICE）"
  if need v4l2-ctl; then
    warn "   现有视频设备："
    v4l2-ctl --list-devices 2>/dev/null | sed 's/^/       /'
  else
    warn "   装 v4l-utils 后可用 v4l2-ctl --list-devices 查看"
  fi
fi

# CAN
if need ip; then
  if ip -details link show can0 >/dev/null 2>&1; then
    ok "CAN can0 存在：$(ip -details link show can0 | head -n 1 | tr -s ' ')"
  else
    warn "未发现 can0。CAN 通信用法：
         sudo modprobe gs_usb                      # USB-CAN 常见驱动
         sudo ip link set can0 up type can bitrate 1000000
         sudo ip link set can0 txqueuelen 1000"
  fi
fi

# 串口
if ls /dev/ttyACM* /dev/ttyUSB* >/dev/null 2>&1; then
  ok "串口设备：$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')"
  [ -e /dev/gimbal ] && ok "udev 软链 /dev/gimbal 已存在" \
    || warn "/dev/gimbal 不存在，但配置里 com_port 用的是它。请手工加 udev 软链（idVendor/idProduct 换成 lsusb 里的实际值），完整命令见 docs/outpost_usbcamera_test_runbook.md §2.3"
else
  warn "未发现 /dev/ttyACM* 或 /dev/ttyUSB*（下位机若走串口通信，插上后重跑本脚本）"
fi

# 分辨率参数只用于和配置交叉核对：USBCamera 的分辨率来自 YAML，不受命令行影响
if [ -f "$CONFIG" ] && grep -qE "^[[:space:]]*image_width:" "$CONFIG"; then
  CFG_W="$(grep -E "^[[:space:]]*image_width:"  "$CONFIG" | head -1 | awk '{print $2}')"
  CFG_H="$(grep -E "^[[:space:]]*image_height:" "$CONFIG" | head -1 | awk '{print $2}')"
  if [ "$CFG_W" = "$VIDEO_WIDTH" ] && [ "$CFG_H" = "$VIDEO_HEIGHT" ]; then
    ok "分辨率一致：配置 ${CFG_W}x${CFG_H}"
  else
    warn "分辨率不一致：命令行 --width/--height = ${VIDEO_WIDTH}x${VIDEO_HEIGHT}，配置 = ${CFG_W}x${CFG_H}"
    warn "  io::USBCamera 用的是配置里的 image_width/image_height，请改配置或用 v4l2-ctl 确认相机支持该分辨率"
  fi
  if need v4l2-ctl && [ -e "$V4L_DEV" ]; then
    if v4l2-ctl -d "$V4L_DEV" --list-formats-ext 2>/dev/null | grep -q "${CFG_W}x${CFG_H}"; then
      ok "相机支持 ${CFG_W}x${CFG_H}"
    else
      warn "相机格式列表里没看到 ${CFG_W}x${CFG_H}，请核对 v4l2-ctl -d $V4L_DEV --list-formats-ext"
    fi
  fi
fi

# 相机厂商标识（UVC 相机可确认插上了）
if need lsusb; then
  if lsusb | grep -qiE 'camera|webcam|video|imaging|hik|mindvision|daheng|galaxy'; then
    ok "lsusb 发现疑似相机：$(lsusb | grep -iE 'camera|webcam|video|imaging|hik|mindvision' | head -n 2 | tr '\n' '|')"
  else
    warn "lsusb 未见明显相机设备（工业相机用厂商 SDK 时也可能不显示）"
  fi
fi

if [ "$PROBE_ONLY" = "1" ]; then
  say ""
  say " --probe-only：环境与设备探测结束"
  [ "$ENV_FAIL" = "0" ] && exit 0 || exit 2
fi

if [ "$ENV_FAIL" != "0" ] && [ "$MODE" = "cmake" ]; then
  warn "环境自检有 FAIL，先尝试用 --standalone 跑算法单测；全量构建需要先补依赖。"
fi

# =============================================================================
# [2/5] 编译
# =============================================================================
say ""
say "[2/5] 编译"

if [ "$DO_CLEAN" = "1" ] && [ -d "$BUILD_DIR" ]; then
  say "  --clean：删除 $BUILD_DIR"
  rm -rf "$BUILD_DIR"
  mkdir -p "$LOG_DIR" "$BIN_DIR"
fi

build_standalone() {
  local cxxflags=(-std=c++17 -O2 -I"$REPO_ROOT" -pthread -I/usr/include/eigen3)
  local covflags=()
  if [ "$WITH_COVERAGE" = "1" ]; then
    covflags=(--coverage -fprofile-arcs -ftest-coverage); cxxflags+=("${covflags[@]}")
  fi
  if pkg-config --exists opencv4; then
    cxxflags+=($(pkg-config --cflags opencv4)); LIBS_OPENCV="$(pkg-config --libs opencv4)"
  else
    cxxflags+=(-I/usr/include/opencv4)
    LIBS_OPENCV="-lopencv_core -lopencv_imgproc -lopencv_calib3d"
  fi
  LIBS_DEPS="$(pkg-config --libs fmt spdlog yaml-cpp 2>/dev/null || echo '-lfmt -lspdlog -lyaml-cpp')"

  local TOOLS_SRC=("$REPO_ROOT/tools/math_tools.cpp" "$REPO_ROOT/tools/trajectory.cpp" "$REPO_ROOT/tools/ballistics.cpp" "$REPO_ROOT/tools/extended_kalman_filter.cpp" "$REPO_ROOT/tools/logger.cpp")
  local AA_COMMON=("$REPO_ROOT/tasks/auto_aim/armor.cpp" "$REPO_ROOT/tasks/auto_aim/outpost_target.cpp")
  local -A SRCS
  SRCS[outpost_helper_test]="$REPO_ROOT/tests/outpost_helper_test.cpp $REPO_ROOT/tools/math_tools.cpp"
  SRCS[outpost_geometry_test]="$REPO_ROOT/tests/outpost_geometry_test.cpp"
  SRCS[outpost_target_test]="$REPO_ROOT/tests/outpost_target_test.cpp ${AA_COMMON[*]} ${TOOLS_SRC[*]}"
  SRCS[outpost_aimer_test]="$REPO_ROOT/tests/outpost_aimer_test.cpp ${AA_COMMON[*]} $REPO_ROOT/tasks/auto_aim/target.cpp $REPO_ROOT/tasks/auto_aim/aimer.cpp ${TOOLS_SRC[*]}"

  local rc=0
  for t in "${TARGETS[@]}"; do
    local objdir="$COV_DIR/$t"; mkdir -p "$objdir"
    echo "  - 编译 $t"
    # shellcheck disable=SC2086
    ( cd "$objdir" && g++ "${cxxflags[@]}" ${SRCS[$t]} -o "$BIN_DIR/$t" $LIBS_OPENCV $LIBS_DEPS ) \
      >"$LOG_DIR/build_$t.log" 2>&1 || { bad "$t 编译失败，日志 $LOG_DIR/build_$t.log"; tail -n 8 "$LOG_DIR/build_$t.log" | sed 's/^/       /'; rc=1; }
  done
  return $rc
}

CMAKE_OK=0
if [ "$MODE" = "cmake" ]; then
  if [ "$OVINO_FOUND" = "1" ]; then
    if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
      say "  cmake -S . -B $BUILD_DIR -DCMAKE_BUILD_TYPE=Release -DOpenVINO_DIR=$OVINO_DIR_HINT"
      if ! cmake -S "$REPO_ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release \
                 -DOpenVINO_DIR="$OVINO_DIR_HINT" >"$LOG_DIR/cmake_configure.log" 2>&1; then
        bad "CMake 配置失败，见 $LOG_DIR/cmake_configure.log"
        grep -iE "error|not found" "$LOG_DIR/cmake_configure.log" | head -n 10 | sed 's/^/       /'
      else
        CMAKE_OK=1
      fi
    else
      say "  复用已有 CMake 缓存：$BUILD_DIR"
      CMAKE_OK=1
    fi
    if [ "$CMAKE_OK" = "1" ]; then
      if cmake --build "$BUILD_DIR" --target "${TARGETS[@]}" -j "$JOBS" \
           >"$LOG_DIR/build_cmake.log" 2>&1; then
        ok "4 个前哨站测试目标编译完成"
      else
        bad "编译失败，见 $LOG_DIR/build_cmake.log"
        tail -n 12 "$LOG_DIR/build_cmake.log" | sed 's/^/       /'
        CMAKE_OK=0
      fi
    fi
  else
    warn "没有可用的 OpenVINO_DIR，cmake 模式跳过，自动回退 standalone"
  fi
fi

BUILD_RC=0
if [ "$MODE" = "standalone" ] || [ "$CMAKE_OK" = "0" ]; then
  [ "$MODE" = "cmake" ] && say "  回退到 standalone（g++ 直编，不依赖 OpenVINO / 相机 SDK）"
  build_standalone || BUILD_RC=1
fi
[ "$BUILD_RC" = "0" ] && ok "编译阶段通过" || bad "编译阶段失败"

exe_path() {
  local t="$1"
  if [ "$CMAKE_OK" != "1" ]; then echo "$BIN_DIR/$t"; return; fi
  for p in "$BUILD_DIR/$t" "$BUILD_DIR/Release/$t" "$BUILD_DIR/bin/$t"; do
    [ -x "$p" ] && { echo "$p"; return; }
  done
  echo ""
}

# =============================================================================
# [3/5] 前哨站离线链路测试
# =============================================================================
say ""
say "[3/5] 前哨站离线链路测试（EKF / 切板 / 板序号 / 快照 / Aimer）"

run_one() {
  local t="$1"; shift
  local exe; exe="$(exe_path "$t")"
  local log="$LOG_DIR/$t.log"
  if [ -z "$exe" ]; then bad "$t: 找不到可执行文件"; record "$t" "MISSING" "无二进制"; return 1; fi
  echo "  - $t"
  ( cd "$REPO_ROOT" && "$exe" "$@" ) >"$log" 2>&1
  local code=$?
  tail -n 4 "$log" | sed 's/^/       /'
  local detail; detail="$(grep -oE '[0-9]+ passed, [0-9]+ failed' "$log" | tail -1)"
  [ -z "$detail" ] && detail="$(grep -oE 'PASS: [0-9]+, FAIL: [0-9]+' "$log" | tail -1)"
  if [ $code -eq 0 ]; then record "$t" "PASS" "${detail:-exit 0}"; return 0
  else record "$t" "FAIL" "${detail:-exit $code}"; return 1; fi
}

TEST_FAILS=0
if [ "$BUILD_RC" != "0" ]; then
  warn "编译未通过，跳过全部测试"
  TEST_FAILS=1
else
  for t in "${TARGETS[@]}"; do
    case "$t" in
      outpost_aimer_test) run_one "$t" "$CONFIG" || TEST_FAILS=$((TEST_FAILS+1)) ;;
      *)                  run_one "$t" || TEST_FAILS=$((TEST_FAILS+1)) ;;
    esac
  done
fi

if [ "$WITH_COVERAGE" = "1" ]; then
  say ""
  say "  [覆盖率] gcovr 统计前哨站相关源文件"
  if need gcovr && [ "$MODE" = "standalone" ]; then
    for t in "${TARGETS[@]}"; do
      gcovr -r "$REPO_ROOT" "$COV_DIR/$t" \
        --filter "$REPO_ROOT/tasks/auto_aim/outpost" \
        --filter "$REPO_ROOT/tasks/auto_aim/aimer" \
        --filter "$REPO_ROOT/tasks/auto_aim/target" --txt 2>/dev/null \
        > "$LOG_DIR/coverage_$t.txt"
    done
    ok "覆盖率文本已写入 $LOG_DIR/coverage_*.txt"
  else
    warn "跳过覆盖率：需要 --standalone + 已安装 gcovr"
  fi
fi

# =============================================================================
# [4/5] 实机链路测试
# =============================================================================
say ""
say "[4/5] 实机链路测试"

REAL_OK=1

# 4.1 外接相机实时识别（YOLO 只做检测，验证“能不能看到前哨站”）
if [ "$WITH_DETECT" = "1" ]; then
  DET_BIN="$BUILD_DIR/usbcamera_detect_test"
  if [ ! -x "$DET_BIN" ]; then
    warn "缺少 $DET_BIN：先做一次全量构建 cmake --build $BUILD_DIR -j$JOBS"
    REAL_OK=0
  else
    ARGS=(-n "$VIDEO_DEVICE" "$CONFIG")
    [ "$WITH_DISPLAY" = "1" ] && ARGS+=(-d)
    if [ "$WITH_DISPLAY" = "1" ]; then
      say "  运行 usbcamera_detect_test（窗口里按 q 退出）"
      ( cd "$REPO_ROOT" && "$DET_BIN" "${ARGS[@]}" 2>&1 | tee "$LOG_DIR/usbcamera_detect.log" )
      rc=${PIPESTATUS[0]}
      [ "$rc" -eq 0 ] && record "usbcamera_detect_test" "PASS" "exit 0" \
                      || record "usbcamera_detect_test" "FAIL" "exit $rc"
    else
      warn "  无显示环境不加 --display；为保证可自动结束，改为限时 15s + 无窗口"
      ( cd "$REPO_ROOT" && timeout 15s "$DET_BIN" "${ARGS[@]}" ) >"$LOG_DIR/usbcamera_detect.log" 2>&1
      rc=$?
      # 相机 -n 参数与 --display 都为可选；这里只用日志判断是否读到帧
      if grep -qE "USBCamera opened|fps" "$LOG_DIR/usbcamera_detect.log"; then
        ok "  外接相机已打开并出帧（日志 $LOG_DIR/usbcamera_detect.log）"
        record "usbcamera_detect_test" "PASS" "见日志"
      else
        bad "  外接相机未出帧（exit $rc），日志尾部："
        tail -n 10 "$LOG_DIR/usbcamera_detect.log" | sed 's/^/       /'
        record "usbcamera_detect_test" "FAIL" "exit $rc"
        REAL_OK=0
      fi
    fi
  fi
else
  warn "  跳过实时识别（加 --detect 开启；弹窗再加 --display）"
fi

# 4.2 录像回放整链路（Tracker + Aimer，最有诊断价值）
if [ -n "$VIDEO_PATH" ]; then
  AA_BIN="$BUILD_DIR/auto_aim_test"
  if [ ! -x "$AA_BIN" ]; then
    warn "缺少 $AA_BIN（需要 OpenVINO + io 相机 SDK 全量构建），跳过录像回放"
    REAL_OK=0
  elif [ ! -f "${VIDEO_PATH}.avi" ] || [ ! -f "${VIDEO_PATH}.txt" ]; then
    bad "  录像需要 ${VIDEO_PATH}.avi 与 ${VIDEO_PATH}.txt（配套 IMU 四元数序列）"
    record "auto_aim_test($VIDEO_PATH)" "FAIL" "缺少 avi/txt"
    REAL_OK=0
  else
    say "  回放 $VIDEO_PATH（前哨站录像同样走 5 维链路）"
    if need xvfb-run; then
      ( cd "$REPO_ROOT" && timeout 180s xvfb-run -a "$AA_BIN" -c "$CONFIG" "$VIDEO_PATH" ) \
        >"$LOG_DIR/auto_aim_test.log" 2>&1
    else
      ( cd "$REPO_ROOT" && timeout 180s "$AA_BIN" -c "$CONFIG" "$VIDEO_PATH" ) \
        >"$LOG_DIR/auto_aim_test.log" 2>&1
    fi
    rc=$?
    DIV="$(grep -c "Target diverged" "$LOG_DIR/auto_aim_test.log" 2>/dev/null)"; DIV="${DIV:-0}"
    if [ "$rc" -eq 0 ]; then
      record "auto_aim_test($VIDEO_PATH)" "PASS" "exit 0, diverged=$DIV"
    else
      record "auto_aim_test($VIDEO_PATH)" "FAIL" "exit $rc（无显示环境请装 xvfb）"
      REAL_OK=0
    fi
    ok "  日志：$LOG_DIR/auto_aim_test.log（关键字见 docs/outpost_usbcamera_test_runbook.md §6）"
  fi
else
  warn "  跳过录像回放（加 --video assets/demo/demo）"
fi

# 4.3 闭环自瞄：默认不自动跑，避免误动云台
cat >"$LOG_DIR/closed_loop_command.txt" <<EOF
# 闭环自瞄（外接相机 + 下位机云台）需要人工确认后手动执行：
#
#   # 1) 先确认模式：下位机 CAN 模式下把档位切到 auto_aim(1) 或 outpost(4)
#   #    串口模式下位机 mode=1 即自瞄
#   # 2) 无人机（有独立前哨站档）：
#   ./build/uav configs/outpost_test.yaml
#   # 3) 步兵（识别到前哨站自动进 5 维链路）：
#   ./build/standard configs/outpost_test.yaml
#   # 4) 只看不下发（无下位机时的纯视觉验证）：把 auto_fire 置 false 并降低云台风险
#
# 停止：在终端按 Ctrl+C（tools::Exiter 会优雅退出）
EOF
if [ "$REAL_OK" = "1" ]; then ok "闭环自瞄命令已备好：$LOG_DIR/closed_loop_command.txt"
else warn "实机测试有待解决项，见上面日志"; fi

# 4.4 可选：地面兵种录像回归（防前哨站改动影响其他兵种）
if [ "$WITH_REGRESSION" = "1" ]; then
  say ""
  say "  [回归] 地面兵种录像 assets/demo"
  AA_BIN="$BUILD_DIR/auto_aim_test"
  if [ -x "$AA_BIN" ] && [ -f assets/demo/demo.avi ]; then
    if need xvfb-run; then
      ( cd "$REPO_ROOT" && timeout 180s xvfb-run -a "$AA_BIN" -c configs/demo.yaml assets/demo/demo ) \
        >"$LOG_DIR/regression_demo.log" 2>&1
    else
      ( cd "$REPO_ROOT" && timeout 180s "$AA_BIN" -c configs/demo.yaml assets/demo/demo ) \
        >"$LOG_DIR/regression_demo.log" 2>&1
    fi
    [ $? -eq 0 ] && record "regression(assets/demo)" "PASS" "exit 0" \
                 || record "regression(assets/demo)" "FAIL" "见 regression_demo.log"
  else
    warn "  跳过回归：需要全量构建的 auto_aim_test 与 assets/demo/demo.avi"
  fi
fi

# =============================================================================
# [5/5] 汇总
# =============================================================================
echo ""
echo "=============================================================="
echo " 汇总"
echo "=============================================================="
FAILS=0
printf " %-34s %-8s %s\n" "项目" "结果" "细节"
for i in "${!R_NAME[@]}"; do
  printf " %-34s %-8s %s\n" "${R_NAME[$i]}" "${R_STATUS[$i]}" "${R_DETAIL[$i]}"
  case "${R_STATUS[$i]}" in FAIL|MISSING) FAILS=$((FAILS+1));; esac
done
echo "--------------------------------------------------------------"
echo " 日志目录：$LOG_DIR"
if [ "$FAILS" -eq 0 ] && [ "$BUILD_RC" = "0" ]; then
  echo " 结果：全部通过（前哨站链路可用）"
  exit 0
else
  echo " 结果：$FAILS 项失败 / 编译状态 $BUILD_RC —— 先看 $LOG_DIR 下对应 *.log"
  exit 1
fi
