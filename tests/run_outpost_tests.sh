#!/usr/bin/env bash
# =============================================================================
# 前哨站（Outpost）相关流程测试脚本
#
# 覆盖对象（与仓库 CMakeLists.txt L105-131 注册的 4 个目标一一对应）：
#   1. outpost_helper_test    PR-1 工具层单测（HeightBuffer / PlateSwitchDetector /
#                             outpost_relative_height / transform_to_reference / 门控）
#   2. outpost_geometry_test  PR-2 邻侧板几何推算（OutpostSolution / NeighborBarSolution）
#   3. outpost_target_test    PR-3 5 维 EKF 集成（真值轨迹 + 噪声，收敛/切板/板序号/门控）
#   4. outpost_aimer_test     PR-6 5 维 → 11 维快照 → Aimer（窗口/滞回/预瞄/退化）
#
# 用法：
#   bash tests/run_outpost_tests.sh                  # 默认：CMake 构建 4 个测试并运行
#   bash tests/run_outpost_tests.sh --standalone     # 不依赖 OpenVINO/相机 SDK，直接用 g++ 编译
#   bash tests/run_outpost_tests.sh --clean          # 先删除 build 目录再配置
#   bash tests/run_outpost_tests.sh --regression     # 额外跑地面兵种录像回归（assets/demo）
#   bash tests/run_outpost_tests.sh --build-dir out/build --config configs/sentry.yaml
#
# 覆盖率（检查“测试对前哨站代码的覆盖是否完整”，仅 standalone 模式）：
#   bash tests/run_outpost_tests.sh --standalone --coverage
#   依赖 gcovr：pip install gcovr  或  sudo apt install gcovr
#
# 参考通过数（本仓库源码在本机 Windows + MSVC 用等价方式实测，Linux 上数值应一致）：
#   outpost_helper_test   : PASS 41,   FAIL 0
#   outpost_geometry_test : 22 passed, 0 failed
#   outpost_target_test   : 28 passed, 0 failed
#   outpost_aimer_test    : 7327 passed, 0 failed（fire_ratio≈0.33）
#
# 退出码：0 = 全部通过；1 = 有测试失败；2 = 构建/环境失败。
# =============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT" || exit 2

MODE="cmake"              # cmake | standalone
BUILD_DIR="$REPO_ROOT/build"
CONFIG="configs/sentry.yaml"
WITH_REGRESSION=0
WITH_COVERAGE=0
DO_CLEAN=0
JOBS="$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"

TARGETS=(outpost_helper_test outpost_geometry_test outpost_target_test outpost_aimer_test)

usage() {
  sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    --standalone) MODE="standalone" ;;
    --coverage)   WITH_COVERAGE=1 ;;
    --clean)      DO_CLEAN=1 ;;
    --regression) WITH_REGRESSION=1 ;;
    --config)     CONFIG="${2:-}"; shift ;;
    --build-dir)  BUILD_DIR="${2:-}"; shift ;;
    -j|--jobs)    JOBS="${2:-}"; shift ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "[ERROR] 未知参数: $1"; usage; exit 2 ;;
  esac
  shift
done

LOG_DIR="$BUILD_DIR/outpost_test_logs"
BIN_DIR="$LOG_DIR/bin"
COV_DIR="$LOG_DIR/cov"
mkdir -p "$LOG_DIR" "$BIN_DIR"

RESULT_NAME=(); RESULT_STATUS=(); RESULT_DETAIL=()
record() { RESULT_NAME+=("$1"); RESULT_STATUS+=("$2"); RESULT_DETAIL+=("$3"); }

echo "=============================================================="
echo " 前哨站流程测试"
echo "   仓库根目录 : $REPO_ROOT"
echo "   构建目录   : $BUILD_DIR"
echo "   模式       : $MODE"
echo "   日志目录   : $LOG_DIR"
echo "=============================================================="

# -----------------------------------------------------------------------------
# 0. 依赖检查
# -----------------------------------------------------------------------------
need() { command -v "$1" >/dev/null 2>&1; }

if [ "$MODE" = "cmake" ]; then
  if ! need cmake; then echo "[ERROR] 找不到 cmake，请先安装：sudo apt install cmake"; exit 2; fi
  if ! need make && ! need ninja; then
    echo "[ERROR] 找不到 make/ninja（sudo apt install build-essential）"; exit 2
  fi
else
  if ! need g++; then echo "[ERROR] 找不到 g++（sudo apt install g++）"; exit 2; fi
fi

# -----------------------------------------------------------------------------
# 1. 构建
# -----------------------------------------------------------------------------
build_cmake() {
  if [ "$DO_CLEAN" = "1" ] && [ -d "$BUILD_DIR" ]; then
    echo "[1/3] --clean：删除 $BUILD_DIR"
    rm -rf "$BUILD_DIR"
    mkdir -p "$LOG_DIR" "$BIN_DIR"
  fi

  if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "[1/3] 配置 CMake：cmake -S . -B $BUILD_DIR"
    if ! cmake -S "$REPO_ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release; then
      cat <<'EOF'

[ERROR] CMake 配置失败。最常见原因是 OpenVINO：
  - 仓库 CMakeLists.txt L15 与 tasks/auto_aim/CMakeLists.txt L3 把 OpenVINO_DIR
    硬编码为 /home/gzy/openvino_env/lib/python3.12/site-packages/openvino/cmake，
    换机器后该路径不存在，find_package(OpenVINO REQUIRED) 会直接失败。
    修改这两处为你自己的 OpenVINO 路径，例如：
      python3 -c 'import openvino,os;print(os.path.join(os.path.dirname(openvino.__file__),"cmake"))'
  - 只想验证前哨站这 4 个测试（它们并不链接 OpenVINO）时，可直接用：
      bash tests/run_outpost_tests.sh --standalone
EOF
      return 1
    fi
  else
    echo "[1/3] 复用已有 CMake 缓存：$BUILD_DIR"
  fi

  echo "[1/3] 编译目标：${TARGETS[*]}"
  if ! cmake --build "$BUILD_DIR" --target "${TARGETS[@]}" -j "$JOBS" 2>&1 | tee "$LOG_DIR/build.log"; then
    echo "[ERROR] 编译失败，完整日志见 $LOG_DIR/build.log"
    echo "        若失败原因是 OpenVINO / io 相机 SDK，可改用：bash tests/run_outpost_tests.sh --standalone"
    return 1
  fi
  return 0
}

build_standalone() {
  echo "[1/3] standalone 模式：直接用 g++ 编译（不依赖 OpenVINO / 相机 SDK）"

  local cxxflags=(-std=c++17 -O2 -I"$REPO_ROOT" -pthread)
  local covflags=()
  if [ "$WITH_COVERAGE" = "1" ]; then
    covflags=(--coverage -fprofile-arcs -ftest-coverage)
    cxxflags+=("${covflags[@]}")
    echo "        覆盖率模式：对象文件输出到 $COV_DIR/<test>/"
  fi
  # Eigen：Ubuntu 下在 /usr/include/eigen3
  cxxflags+=(-I/usr/include/eigen3)
  if need pkg-config && pkg-config --exists opencv4; then
    # shellcheck disable=SC2207
    cxxflags+=($(pkg-config --cflags opencv4))
    LIBS_OPENCV="$(pkg-config --libs opencv4)"
  else
    cxxflags+=(-I/usr/include/opencv4)
    LIBS_OPENCV="-lopencv_core -lopencv_imgproc -lopencv_calib3d"
  fi
  if need pkg-config; then
    LIBS_DEPS="$(pkg-config --libs fmt spdlog yaml-cpp 2>/dev/null || echo '-lfmt -lspdlog -lyaml-cpp')"
  else
    LIBS_DEPS="-lfmt -lspdlog -lyaml-cpp"
  fi

  # 与仓库 tools 目标对应的最小源文件集合（4 个测试实际用到的部分）
  local TOOLS_SRC=(
    "$REPO_ROOT/tools/math_tools.cpp"
    "$REPO_ROOT/tools/trajectory.cpp"
    "$REPO_ROOT/tools/extended_kalman_filter.cpp"
    "$REPO_ROOT/tools/logger.cpp"
  )
  local AUTO_AIM_COMMON=(
    "$REPO_ROOT/tasks/auto_aim/armor.cpp"
    "$REPO_ROOT/tasks/auto_aim/outpost_target.cpp"
  )

  local -A SRCS
  SRCS[outpost_helper_test]="$REPO_ROOT/tests/outpost_helper_test.cpp $REPO_ROOT/tools/math_tools.cpp"
  SRCS[outpost_geometry_test]="$REPO_ROOT/tests/outpost_geometry_test.cpp"
  SRCS[outpost_target_test]="$REPO_ROOT/tests/outpost_target_test.cpp ${AUTO_AIM_COMMON[*]} ${TOOLS_SRC[*]}"
  SRCS[outpost_aimer_test]="$REPO_ROOT/tests/outpost_aimer_test.cpp ${AUTO_AIM_COMMON[*]} \
$REPO_ROOT/tasks/auto_aim/target.cpp $REPO_ROOT/tasks/auto_aim/aimer.cpp ${TOOLS_SRC[*]}"

  local ok=0
  for t in "${TARGETS[@]}"; do
    echo "  - 编译 $t"
    local objdir="$COV_DIR/$t"
    mkdir -p "$objdir"
    # shellcheck disable=SC2086
    if ! ( cd "$objdir" && g++ "${cxxflags[@]}" ${SRCS[$t]} -o "$BIN_DIR/$t" $LIBS_OPENCV $LIBS_DEPS ) \
         2>&1 | tee "$LOG_DIR/build_$t.log"; then
      echo "[ERROR] $t 编译失败，日志见 $LOG_DIR/build_$t.log"
      ok=1
    fi
  done
  return $ok
}

BUILD_OK=1
if [ "$MODE" = "cmake" ]; then
  build_cmake && BUILD_OK=0
else
  build_standalone && BUILD_OK=0
fi

if [ "$BUILD_OK" != "0" ]; then
  echo "[FATAL] 构建未通过，无法运行测试。"
  exit 2
fi

# -----------------------------------------------------------------------------
# 2. 运行测试
# -----------------------------------------------------------------------------
exe_path() {
  local t="$1"
  if [ "$MODE" = "standalone" ]; then
    echo "$BIN_DIR/$t"
  else
    # 兼容单配置（build/<t>）与多配置（build/Release/<t>）生成器
    for p in "$BUILD_DIR/$t" "$BUILD_DIR/Release/$t" "$BUILD_DIR/bin/$t"; do
      [ -x "$p" ] && { echo "$p"; return; }
    done
    echo ""
  fi
}

echo
echo "[2/3] 运行测试"
for t in "${TARGETS[@]}"; do
  exe="$(exe_path "$t")"
  log="$LOG_DIR/$t.log"
  if [ -z "$exe" ]; then
    echo "  - $t : [缺失可执行文件]"
    record "$t" "MISSING" "未找到可执行文件"
    continue
  fi

  echo "  - $t"
  # outpost_aimer_test 支持 argv[1] 指定 Aimer 的 YAML 配置（默认 configs/sentry.yaml）
  if [ "$t" = "outpost_aimer_test" ]; then
    if [ ! -f "$REPO_ROOT/$CONFIG" ]; then
      echo "      [WARN] 配置不存在，回退默认值：$CONFIG"
      ( cd "$REPO_ROOT" && "$exe" ) >"$log" 2>&1
    else
      ( cd "$REPO_ROOT" && "$exe" "$CONFIG" ) >"$log" 2>&1
    fi
  else
    ( cd "$REPO_ROOT" && "$exe" ) >"$log" 2>&1
  fi
  code=$?
  tail -n 6 "$log" | sed 's/^/      /'

  # 从输出里抽取通过/失败计数（各测试的打印格式不同，尽量兼容）
  detail="$(grep -oE '[0-9]+ passed, [0-9]+ failed' "$log" | tail -1)"
  [ -z "$detail" ] && detail="$(grep -oE 'PASS: [0-9]+, FAIL: [0-9]+' "$log" | tail -1)"

  if [ $code -eq 0 ]; then
    record "$t" "PASS" "${detail:-exit 0}"
  else
    record "$t" "FAIL" "${detail:-exit $code}"
  fi
done

# -----------------------------------------------------------------------------
# 3. 覆盖率（可选，仅 standalone 模式）
# -----------------------------------------------------------------------------
if [ "$WITH_COVERAGE" = "1" ]; then
  echo
  echo "[覆盖率] 统计前哨站相关源文件的行覆盖（gcovr）"
  if [ "$MODE" != "standalone" ]; then
    echo "  - 跳过：--coverage 仅支持 --standalone 模式"
    record "coverage" "SKIP" "需要 --standalone"
  elif command -v gcovr >/dev/null 2>&1; then
    for t in "${TARGETS[@]}"; do
      echo "  --- $t ---"
      gcovr -r "$REPO_ROOT" "$COV_DIR/$t" \
        --filter "$REPO_ROOT/tasks/auto_aim/outpost" \
        --filter "$REPO_ROOT/tasks/auto_aim/aimer" \
        --filter "$REPO_ROOT/tasks/auto_aim/target" \
        --filter "$REPO_ROOT/tasks/auto_aim/tracker" \
        --filter "$REPO_ROOT/tasks/auto_aim/solver" \
        --gcov-ignore-parse-errors --txt 2>&1 \
        | tee "$LOG_DIR/coverage_$t.txt" | tail -n 20
    done
    echo "  说明：以上是各测试单独的覆盖，未跨测试合并；Tracker / Solver 的"
    echo "        前哨站分支目前没有单元测试覆盖（见 docs/outpost_flow_summary.md §6）。"
    record "coverage(gcovr)" "INFO" "$LOG_DIR/coverage_*.txt"
  else
    echo "  - 未找到 gcovr：pip install gcovr  或  sudo apt install gcovr"
    record "coverage(gcovr)" "SKIP" "gcovr 未安装"
  fi
fi

# -----------------------------------------------------------------------------
# 4. 可选：地面兵种录像回归（需要完整 auto_aim 库 + OpenVINO）
# -----------------------------------------------------------------------------
if [ "$WITH_REGRESSION" = "1" ]; then
  echo
  echo "[回归] 地面兵种录像回归（assets/demo，需 OpenVINO）"
  if [ "$MODE" = "standalone" ]; then
    echo "  - 跳过：standalone 模式不构建识别器/OpenVINO 相关目标"
    record "auto_aim_test(regression)" "SKIP" "standalone 模式不支持"
  else
    if cmake --build "$BUILD_DIR" --target auto_aim_test -j "$JOBS" >"$LOG_DIR/build_auto_aim_test.log" 2>&1; then
      if [ -f "$REPO_ROOT/assets/demo/demo.avi" ]; then
        # 注意：Plotter 会尝试创建窗口；无显示环境请用 xvfb-run 包裹
        if command -v xvfb-run >/dev/null 2>&1; then
          ( cd "$REPO_ROOT" && xvfb-run -a ./build/auto_aim_test -c "$CONFIG" assets/demo/demo ) \
            >"$LOG_DIR/auto_aim_test.log" 2>&1
        else
          ( cd "$REPO_ROOT" && ./build/auto_aim_test -c "$CONFIG" assets/demo/demo ) \
            >"$LOG_DIR/auto_aim_test.log" 2>&1
        fi
        code=$?
        [ $code -eq 0 ] && record "auto_aim_test(regression)" "PASS" "exit 0" \
                        || record "auto_aim_test(regression)" "FAIL" "exit $code（无显示环境可用 xvfb-run）"
      else
        record "auto_aim_test(regression)" "SKIP" "缺少 assets/demo/demo.avi"
      fi
    else
      record "auto_aim_test(regression)" "FAIL" "构建失败，见 $LOG_DIR/build_auto_aim_test.log"
    fi
  fi
fi

# -----------------------------------------------------------------------------
# 5. 汇总
# -----------------------------------------------------------------------------
echo
echo "=============================================================="
echo " 汇总"
echo "=============================================================="
printf " %-34s %-8s %s\n" "测试" "结果" "细节"
fails=0
for i in "${!RESULT_NAME[@]}"; do
  printf " %-34s %-8s %s\n" "${RESULT_NAME[$i]}" "${RESULT_STATUS[$i]}" "${RESULT_DETAIL[$i]}"
  case "${RESULT_STATUS[$i]}" in FAIL|MISSING) fails=$((fails + 1));; esac
done
echo "--------------------------------------------------------------"
echo " 日志目录：$LOG_DIR"
if [ "$fails" -eq 0 ]; then
  echo " 结果：全部通过 ✅"
  exit 0
else
  echo " 结果：$fails 项失败 ❌"
  exit 1
fi
