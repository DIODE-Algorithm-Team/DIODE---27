// 前哨站工具层单元测试 (PR-1)
//
// 覆盖：
//   1. HeightBuffer 板序号辨识（单板/两板/三板/清理）
//   2. PlateSwitchDetector 连续两次同向确认
//   3. outpost_relative_height 四象限表
//   4. transform_to_reference 参考板坐标系折算
//   5. is_observation_valid 近/远距离门控
//
// 编译：cmake 构建 outpost_helper_test 目标，运行可执行即可。

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "tasks/auto_aim/outpost_const.hpp"
#include "tasks/auto_aim/outpost_helper.hpp"

using auto_aim::HeightBuffer;
using auto_aim::PlateSwitchDetector;
using auto_aim::is_observation_valid;
using auto_aim::outpost_relative_height;
using auto_aim::transform_to_reference;
using auto_aim::outpost_const::kAngleStep;
using auto_aim::outpost_const::kHeightStep;

namespace
{
int g_pass = 0;
int g_fail = 0;

#define CHECK_NEAR(actual, expected, tol, msg)                                       \
  do {                                                                                \
    double _a = static_cast<double>(actual);                                          \
    double _e = static_cast<double>(expected);                                        \
    if (std::fabs(_a - _e) <= (tol)) {                                                \
      ++g_pass;                                                                       \
    } else {                                                                          \
      ++g_fail;                                                                       \
      std::printf("FAIL %s: actual=%.6f expected=%.6f tol=%.6f\n", (msg), _a, _e,      \
                  static_cast<double>(tol));                                          \
    }                                                                                 \
  } while (0)

#define CHECK_EQ(actual, expected, msg)                                               \
  do {                                                                                \
    long _a = static_cast<long>(actual);                                             \
    long _e = static_cast<long>(expected);                                            \
    if (_a == _e) {                                                                  \
      ++g_pass;                                                                       \
    } else {                                                                          \
      ++g_fail;                                                                       \
      std::printf("FAIL %s: actual=%ld expected=%ld\n", (msg), _a, _e);               \
    }                                                                                 \
  } while (0)

#define CHECK_TRUE(actual, msg)                                                       \
  do {                                                                                \
    if ((actual)) {                                                                   \
      ++g_pass;                                                                       \
    } else {                                                                          \
      ++g_fail;                                                                       \
      std::printf("FAIL %s: expected true\n", (msg));                                \
    }                                                                                 \
  } while (0)

#define CHECK_FALSE(actual, msg)                                                      \
  do {                                                                                \
    if (!(actual)) {                                                                  \
      ++g_pass;                                                                       \
    } else {                                                                          \
      ++g_fail;                                                                       \
      std::printf("FAIL %s: expected false\n", (msg));                               \
    }                                                                                 \
  } while (0)

// ============ HeightBuffer 测试 ============

// 仅观测一块板：所有邻居为 NaN，is_upper 全 true → level=0
void test_height_buffer_single_plate()
{
  HeightBuffer hb;
  CHECK_FALSE(hb.all_observed(), "single_plate.all_observed_initial");

  hb.update(1.0);  // 当前 index=0，板0高
  CHECK_EQ(hb.level(), 0, "single_plate.level_at_0");
  CHECK_FALSE(hb.all_observed(), "single_plate.all_observed_after_one");
}

// 观测两块板：板0=1.0（高）、板1=0.9（中）
// go_next(+1) 后 index=1，此时板2 仍 NaN
void test_height_buffer_two_plates()
{
  HeightBuffer hb;
  hb.update(1.0);           // 板0
  hb.go_next(+1);            // index=1
  hb.update(0.9);           // 板1

  // 板1 level：is_lower(0.9, 1.0)=true, is_upper(0.9, NaN)=true → 中(1)
  CHECK_EQ(hb.level(), 1, "two_plates.level_at_1");
  CHECK_FALSE(hb.all_observed(), "two_plates.all_observed");
}

// 三块板齐：板0=1.0 / 板1=0.9 / 板2=0.8，分别应为 高/中/低
void test_height_buffer_three_plates()
{
  HeightBuffer hb;
  hb.update(1.0);           // 板0
  hb.go_next(+1);            // index=1
  hb.update(0.9);           // 板1
  hb.go_next(+1);            // index=2
  hb.update(0.8);           // 板2

  CHECK_TRUE(hb.all_observed(), "three_plates.all_observed");

  // 当前 index=2，level 应为 低(2)
  CHECK_EQ(hb.level(), 2, "three_plates.level_at_2");

  // 用 level_at 检查每个槽位的序号
  CHECK_EQ(hb.level_at(0), 0, "three_plates.level_at(0)");
  CHECK_EQ(hb.level_at(1), 1, "three_plates.level_at(1)");
  CHECK_EQ(hb.level_at(2), 2, "three_plates.level_at(2)");
}

// clean_oldest 在三块板齐后应清理最旧板
// 接三块板场景，last_sign=+1，clean_oldest 清理 data[(2 - 2 + 9) % 3] = data[0]
void test_height_buffer_clean_oldest()
{
  HeightBuffer hb;
  hb.update(1.0);
  hb.go_next(+1);
  hb.update(0.9);
  hb.go_next(+1);
  hb.update(0.8);

  CHECK_TRUE(hb.all_observed(), "clean_oldest.all_observed_before");
  hb.clean_oldest();
  CHECK_FALSE(hb.all_observed(), "clean_oldest.all_observed_after");
  // 最旧板（data[0]）应被清为 NaN
  CHECK_TRUE(std::isnan(hb.height_at(0)), "clean_oldest.data[0]_is_nan");
}

// ============ PlateSwitchDetector 测试 ============

// 连续两次同向 +1 才锁定
void test_plate_switch_confirm_count()
{
  PlateSwitchDetector d;
  CHECK_EQ(d.update(+1), 0, "confirm.1st_plus_one_unconfirmed");
  CHECK_EQ(d.update(+1), +1, "confirm.2nd_plus_one_locked");

  // 重置后做反向序列
  PlateSwitchDetector d2;
  CHECK_EQ(d2.update(-1), 0, "confirm.1st_minus_one_unconfirmed");
  CHECK_EQ(d2.update(+1), 0, "confirm.flip_to_plus_pending");
  CHECK_EQ(d2.update(+1), +1, "confirm.2nd_plus_one_locked_after_flip");
}

// update(0) 返回当前锁定方向（不改变状态）
void test_plate_switch_zero_input()
{
  PlateSwitchDetector d;
  CHECK_EQ(d.update(0), 0, "zero.before_any_lock");
  d.update(+1);
  d.update(+1);  // 锁定 +1
  CHECK_EQ(d.update(0), +1, "zero.after_lock_returns_locked");
  CHECK_EQ(d.update(0), +1, "zero.still_returns_locked");
}

// ============ outpost_relative_height 四象限 ============

void test_outpost_relative_height()
{
  CHECK_NEAR(outpost_relative_height(true, true), +2.0 * kHeightStep, 1e-12, "rel_h.RU");
  CHECK_NEAR(outpost_relative_height(true, false), -1.0 * kHeightStep, 1e-12, "rel_h.RD");
  CHECK_NEAR(outpost_relative_height(false, true), +1.0 * kHeightStep, 1e-12, "rel_h.LU");
  CHECK_NEAR(outpost_relative_height(false, false), -2.0 * kHeightStep, 1e-12, "rel_h.LD");
}

// ============ transform_to_reference 折算 ============

// rel_see_index=0 → 不折算
void test_transform_identity()
{
  Eigen::Vector4d obs(1.0, 0.0, 1.0, 0.5);
  auto before = obs;
  transform_to_reference(obs, 0, true, true);
  CHECK_NEAR(obs(0), before(0), 1e-12, "tr.id.x");
  CHECK_NEAR(obs(1), before(1), 1e-12, "tr.id.y");
  CHECK_NEAR(obs(2), before(2), 1e-12, "tr.id.z");
  CHECK_NEAR(obs(3), before(3), 1e-12, "tr.id.yaw");
}

// rel_see_index=+1, in_right=true, in_upper=true
// delta_yaw = +2π/3, delta_height = +2*step
// obs.z -= +2*step, obs.yaw = limit_rad(obs.yaw - 2π/3)
void test_transform_right_upper()
{
  Eigen::Vector4d obs(1.0, 0.0, 1.0, 0.0);
  transform_to_reference(obs, +1, true, true);
  CHECK_NEAR(obs(0), 1.0, 1e-12, "tr.RU.x");
  CHECK_NEAR(obs(1), 0.0, 1e-12, "tr.RU.y");
  CHECK_NEAR(obs(2), 1.0 - 2.0 * kHeightStep, 1e-12, "tr.RU.z");
  // limit_rad(0 - 2π/3) = -2π/3（在 (-π, π] 内）
  CHECK_NEAR(obs(3), -kAngleStep, 1e-12, "tr.RU.yaw");
}

// rel_see_index=-1, in_right=false, in_upper=false
// delta_yaw = -2π/3, delta_height = -2*step
// obs.z -= -2*step = +2*step, obs.yaw = limit_rad(obs.yaw - (-2π/3)) = limit_rad(+2π/3) = +2π/3
void test_transform_left_lower()
{
  Eigen::Vector4d obs(1.0, 0.0, 1.0, 0.0);
  transform_to_reference(obs, -1, false, false);
  CHECK_NEAR(obs(0), 1.0, 1e-12, "tr.LL.x");
  CHECK_NEAR(obs(1), 0.0, 1e-12, "tr.LL.y");
  CHECK_NEAR(obs(2), 1.0 + 2.0 * kHeightStep, 1e-12, "tr.LL.z");
  CHECK_NEAR(obs(3), +kAngleStep, 1e-12, "tr.LL.yaw");
}

// ============ is_observation_valid 门控 ============

// 近距离（<5m）阈值 0.10，xy_error=0.04 → 有效
void test_obs_valid_near_pass()
{
  Eigen::Vector4d obs(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector4d last(1.04, 0.0, 0.0, 0.0);  // xy_error=0.04
  CHECK_TRUE(is_observation_valid(obs, last), "obs.near_pass");
}

// 近距离，xy_error=0.15 > 0.10 → 无效
void test_obs_valid_near_fail()
{
  Eigen::Vector4d obs(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector4d last(1.15, 0.0, 0.0, 0.0);  // xy_error=0.15
  CHECK_FALSE(is_observation_valid(obs, last), "obs.near_fail");
}

// 远距离（>5m）阈值 0.20，xy_error=0.15 → 有效
void test_obs_valid_far_pass()
{
  Eigen::Vector4d obs(6.0, 0.0, 0.0, 0.0);
  Eigen::Vector4d last(6.15, 0.0, 0.0, 0.0);  // xy_error=0.15
  CHECK_TRUE(is_observation_valid(obs, last), "obs.far_pass");
}

// 远距离，xy_error=0.25 > 0.20 → 无效
void test_obs_valid_far_fail()
{
  Eigen::Vector4d obs(6.0, 0.0, 0.0, 0.0);
  Eigen::Vector4d last(6.25, 0.0, 0.0, 0.0);  // xy_error=0.25
  CHECK_FALSE(is_observation_valid(obs, last), "obs.far_fail");
}

}  // namespace

int main()
{
  std::printf("===== OutpostHelper Tests (PR-1) =====\n");

  test_height_buffer_single_plate();
  test_height_buffer_two_plates();
  test_height_buffer_three_plates();
  test_height_buffer_clean_oldest();
  test_plate_switch_confirm_count();
  test_plate_switch_zero_input();
  test_outpost_relative_height();
  test_transform_identity();
  test_transform_right_upper();
  test_transform_left_lower();
  test_obs_valid_near_pass();
  test_obs_valid_near_fail();
  test_obs_valid_far_pass();
  test_obs_valid_far_fail();

  std::printf("----------------------------------------\n");
  std::printf("PASS: %d, FAIL: %d\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
