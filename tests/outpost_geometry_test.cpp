// 前哨站邻侧板几何推算单元测试
// 移植自 rmcs `OutpostSolution` / `NeighborBarSolution`（utility/math/outpost.cpp）
//
// 验证项：
//   1. 同源 (source == target) 时输出 == 输入（含姿态等价）
//   2. 旋转中心水平偏移 == radius + thickness
//   3. UPPER→MIDDLE 目标板位置符合 Rz(120°) 旋转 + 高度差解析值
//   4. 三个目标板 (UPPER/MIDDLE/LOWER) 距旋转中心水平距离相等
//   5. NeighborBarSolution 的 near 端点比 away 更近
//   6. 邻侧灯条上下端点 z 差 == 灯条高度 56mm
//   7. get_level 四象限表与 rmcs 一致
//   8. 切板循环 UPPER→MIDDLE→LOWER→UPPER 位置闭合（姿态传播正确性，
//      可捕获旋转矩阵行/列构造错误）

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <vector>

#include "../tasks/auto_aim/outpost_const.hpp"
#include "../tasks/auto_aim/outpost_geometry.hpp"

using namespace auto_aim;

namespace
{
constexpr double kPi = outpost_const::kPi;
constexpr double kDeg2Rad = kPi / 180.0;

/// 构造 outpost 装甲板位姿：yaw=0, pitch=-15°, roll=0
/// 板中心位于 (5, 0, 0) - 相机前方 5m，正对相机
struct OutpostPose {
  Eigen::Vector3d translation;
  Eigen::Quaterniond orientation;
};

OutpostPose make_pose(double x, double y, double z, double yaw_deg)
{
  const double yaw = yaw_deg * kDeg2Rad;
  const double pitch = outpost_const::kPredictedPitch;  // -15°
  // ZYX intrinsic: yaw(Z) → pitch(Y) → roll(X=0)
  Eigen::Vector3d ypr(yaw, pitch, 0.0);
  // 复用 SP 风格：tools::rotation_matrix 的等价实现
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  Eigen::Matrix3d R;
  R << cy * cp, -sy, cy * sp, sy * cp, cy, sy * sp, -sp, 0, cp;
  return {Eigen::Vector3d(x, y, z), Eigen::Quaterniond(R).normalized()};
}

bool approx_eq(double a, double b, double eps = 1e-6) { return std::abs(a - b) < eps; }

bool vec_approx_eq(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double eps = 1e-6)
{
  return (a - b).norm() < eps;
}

int passed = 0;
int failed = 0;

#define EXPECT_NEAR(a, b, eps)                                              \
  do {                                                                      \
    if (!approx_eq((a), (b), (eps))) {                                      \
      std::printf(                                                          \
        "FAIL [%s:%d] expected %.9f got %.9f (eps=%.2e)\n", __FILE__,       \
        __LINE__, static_cast<double>(b), static_cast<double>(a),           \
        static_cast<double>(eps));                                          \
      ++failed;                                                             \
    } else {                                                                \
      ++passed;                                                             \
    }                                                                       \
  } while (0)

#define EXPECT_VEC_NEAR(a, b, eps)                                          \
  do {                                                                      \
    if (!vec_approx_eq((a), (b), (eps))) {                                  \
      std::printf(                                                          \
        "FAIL [%s:%d] vector mismatch (%.6f,%.6f,%.6f) vs (%.6f,%.6f,%.6f)\n", \
        __FILE__, __LINE__, (a).x(), (a).y(), (a).z(), (b).x(), (b).y(),    \
        (b).z());                                                          \
      ++failed;                                                             \
    } else {                                                                \
      ++passed;                                                             \
    }                                                                       \
  } while (0)

/// 测试 1：同源同目 (source == target) 时输出应等于输入
void test_identity()
{
  auto pose = make_pose(5.0, 0.0, 0.0, 0.0);
  OutpostSolution sol;
  sol.input.translation = pose.translation;
  sol.input.orientation = pose.orientation;
  sol.input.source = OutpostArmorLevel::UPPER;
  sol.input.target = OutpostArmorLevel::UPPER;
  sol.input.armor_thickness = 0.0;
  sol.solve();

  EXPECT_VEC_NEAR(sol.result.translation, pose.translation, 1e-9);
  // orientation 在 quaternion 等价意义下相等
  EXPECT_NEAR(sol.result.orientation.angularDistance(pose.orientation), 0.0, 1e-9);
}

/// 测试 2：旋转中心 = 输入板位置 + (radius + thickness) * armor2center
///         对 yaw=0, pitch=-15°，armor2center ≈ (1, 0, 0)
void test_rotation_center_offset()
{
  const double thickness = 0.003;
  const double expected_radius = outpost_const::kRadius + thickness;
  auto pose = make_pose(5.0, 0.0, 0.0, 0.0);

  OutpostSolution sol;
  sol.input.translation = pose.translation;
  sol.input.orientation = pose.orientation;
  sol.input.source = OutpostArmorLevel::UPPER;
  sol.input.target = OutpostArmorLevel::UPPER;
  sol.input.armor_thickness = thickness;
  sol.solve();

  EXPECT_NEAR(sol.result.center.x(), 5.0 + expected_radius, 1e-9);
  EXPECT_NEAR(sol.result.center.y(), 0.0, 1e-9);
  EXPECT_NEAR(sol.result.center.z(), 0.0, 1e-9);
}

/// 测试 3：旋转轴方向近似 (0, 0, 1)
///         通过 UPPER → MIDDLE 的目标板位置反推：
///         target_center = center + R(120°, vertical) * (-armor2center * r) + vertical * h
///         取 -armor2center 方向（即 -x）做旋转后，垂直分量来自 vertical
void test_vertical_axis()
{
  auto pose = make_pose(5.0, 0.0, 0.0, 0.0);
  OutpostSolution sol;
  sol.input.translation = pose.translation;
  sol.input.orientation = pose.orientation;
  sol.input.source = OutpostArmorLevel::UPPER;
  sol.input.target = OutpostArmorLevel::MIDDLE;
  sol.input.armor_thickness = 0.0;
  sol.solve();

  // target_center.x - center.x ≈ 0.5 * radius（cos(120°)=-0.5，向量 (-r,0,0) 绕 Z 转 120°）
  //   Rz(120°) * (-r, 0, 0) = (-r*cos120°, -r*sin120°, 0) = (+0.5r, -0.866r, 0)
  const double radius = outpost_const::kRadius;
  const double h_diff = -1.0 * outpost_const::kHeightStep;
  // 期望垂直方向分量 ≈ 1：target_center.z - 0 ≈ h_diff * 1
  EXPECT_NEAR(sol.result.translation.z(), h_diff, 1e-9);
  // 期望 x 方向偏移：-radius * cos(120°) = +0.5 * radius
  EXPECT_NEAR(sol.result.translation.x() - sol.result.center.x(), 0.5 * radius, 1e-9);
  // 期望 y 方向偏移：-radius * sin(120°) = -0.866 * radius
  EXPECT_NEAR(
    sol.result.translation.y() - sol.result.center.y(),
    -std::sqrt(3.0) / 2.0 * radius, 1e-9);
}

/// 测试 4：三个目标板 (UPPER→MIDDLE→LOWER→UPPER) 距旋转中心水平距离相等
void test_three_plates_circular()
{
  auto pose = make_pose(5.0, 0.0, 0.0, 0.0);
  const double radius = outpost_const::kRadius;

  struct Case {
    OutpostArmorLevel source;
    OutpostArmorLevel target;
  };
  const std::vector<Case> cases = {
    {OutpostArmorLevel::UPPER, OutpostArmorLevel::MIDDLE},
    {OutpostArmorLevel::MIDDLE, OutpostArmorLevel::LOWER},
    {OutpostArmorLevel::LOWER, OutpostArmorLevel::UPPER},
  };

  for (const auto & c : cases) {
    OutpostSolution sol;
    sol.input.translation = pose.translation;
    sol.input.orientation = pose.orientation;
    sol.input.source = c.source;
    sol.input.target = c.target;
    sol.input.armor_thickness = 0.0;
    sol.solve();

    const Eigen::Vector3d & center = sol.result.center;
    const Eigen::Vector3d & target = sol.result.translation;
    const double dx = target.x() - center.x();
    const double dy = target.y() - center.y();
    const double horiz_dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(horiz_dist, radius, 1e-9);
  }
}

/// 测试 5：NeighborBarSolution 的 near 端点距当前板更近
void test_neighbor_near_closer_than_away()
{
  auto pose = make_pose(5.0, 0.0, 0.0, 0.0);

  for (const bool is_right : {true, false}) {
    for (const bool is_upper : {true, false}) {
      NeighborBarSolution nb;
      nb.input.translation = pose.translation;
      nb.input.orientation = pose.orientation;
      nb.input.in_right = is_right;
      nb.input.armor_thickness = 0.0;
      nb.solve();

      const auto & near = is_upper ? nb.result.upper_near : nb.result.lower_near;
      const auto & away = is_upper ? nb.result.upper_away : nb.result.lower_away;
      const double near_dist = (near.upper - pose.translation).norm();
      const double away_dist = (away.upper - pose.translation).norm();
      // near 应严格小于 away
      if (!(near_dist < away_dist)) {
        std::printf(
          "FAIL near >= away (is_right=%d, is_upper=%d): near=%.4f away=%.4f\n",
          is_right, is_upper, near_dist, away_dist);
        ++failed;
      } else {
        ++passed;
      }
    }
  }
}

/// 测试 6：邻侧板上下灯条端点的 z 差 == lightbar height (56mm)
void test_neighbor_lightbar_height()
{
  auto pose = make_pose(5.0, 0.0, 0.0, 0.0);
  constexpr double kExpectedHeight = 0.056;

  NeighborBarSolution nb;
  nb.input.translation = pose.translation;
  nb.input.orientation = pose.orientation;
  nb.input.in_right = true;
  nb.input.armor_thickness = 0.0;
  nb.solve();

  // upper_near 的 upper/lower 端点距离应 == 56mm
  const double upper_diff =
    (nb.result.upper_near.upper - nb.result.upper_near.lower).norm();
  EXPECT_NEAR(upper_diff, kExpectedHeight, 1e-9);

  const double lower_diff =
    (nb.result.lower_near.upper - nb.result.lower_near.lower).norm();
  EXPECT_NEAR(lower_diff, kExpectedHeight, 1e-9);
}

/// 测试 7：get_level 四象限表与 outpost_relative_height 一致
void test_get_level_consistency()
{
  using Lvl = OutpostArmorLevel;
  // (is_right, is_upper) → (source, target)
  struct Case {
    bool is_right, is_upper;
    Lvl src, tgt;
  };
  const std::vector<Case> cases = {
    {true, true, Lvl::LOWER, Lvl::UPPER},
    {true, false, Lvl::UPPER, Lvl::MIDDLE},
    {false, true, Lvl::MIDDLE, Lvl::UPPER},
    {false, false, Lvl::UPPER, Lvl::LOWER},
  };
  for (const auto & c : cases) {
    const auto [s, t] = OutpostSolution::get_level(c.is_right, c.is_upper);
    if (s != c.src || t != c.tgt) {
      std::printf(
        "FAIL get_level(%d,%d) = (%d,%d) expected (%d,%d)\n", c.is_right,
        c.is_upper, static_cast<int>(s), static_cast<int>(t),
        static_cast<int>(c.src), static_cast<int>(c.tgt));
      ++failed;
    } else {
      ++passed;
    }
  }
}

/// 测试 8：切板循环 UPPER→MIDDLE→LOWER→UPPER 后回到原位（带数值容差）
void test_three_step_loop_returns()
{
  auto pose = make_pose(5.0, 0.0, 0.0, 0.0);

  OutpostSolution sol;
  sol.input.translation = pose.translation;
  sol.input.orientation = pose.orientation;
  sol.input.armor_thickness = 0.0;

  // 三次连续 UPPER→MIDDLE→LOWER→UPPER，每次用上一次的输出作为下一次输入
  Eigen::Vector3d current_t = pose.translation;
  Eigen::Quaterniond current_q = pose.orientation;
  OutpostArmorLevel current_lvl = OutpostArmorLevel::UPPER;

  const std::vector<OutpostArmorLevel> targets = {
    OutpostArmorLevel::MIDDLE, OutpostArmorLevel::LOWER, OutpostArmorLevel::UPPER};

  for (const auto & target : targets) {
    sol.input.translation = current_t;
    sol.input.orientation = current_q;
    sol.input.source = current_lvl;
    sol.input.target = target;
    sol.solve();
    current_t = sol.result.translation;
    current_q = sol.result.orientation;
    current_lvl = target;
  }

  // 经过完整循环 120°+120°+120°=360° + 高度差 (-1-1-1+2)*step = -1*step
  // 应当回到原始水平位置（高度可能因为 step=0 但累计路径不为 0）
  // UPPER→MIDDLE: -1 step；MIDDLE→LOWER: -1 step；LOWER→UPPER: +2 step；累计 0
  EXPECT_VEC_NEAR(current_t, pose.translation, 1e-6);
}

}  // namespace

int main()
{
  test_identity();
  test_rotation_center_offset();
  test_vertical_axis();
  test_three_plates_circular();
  test_neighbor_near_closer_than_away();
  test_neighbor_lightbar_height();
  test_get_level_consistency();
  test_three_step_loop_returns();

  std::printf("outpost_geometry_test: %d passed, %d failed\n", passed, failed);
  return failed == 0 ? 0 : 1;
}
