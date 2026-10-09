#ifndef AUTO_AIM__OUTPOST_GEOMETRY_HPP
#define AUTO_AIM__OUTPOST_GEOMETRY_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tuple>

#include "outpost_const.hpp"

namespace auto_aim
{
/// @brief 板高度等级（高 / 中 / 低）
enum class OutpostArmorLevel { UPPER, MIDDLE, LOWER };

/// @brief 邻侧板几何推算
///
/// 移植自 rmcs `OutpostSolution`（utility/math/outpost.cpp L50-86）。
/// 输入当前板位姿（任意统一参考系：世界系或相机系）+ 源/目标板等级 +
/// 装甲板厚度，输出目标板中心位置 + 朝向（同一参考系）。
///
/// 几何要点：
/// - 当前板 X 轴（armor→center 方向）绕自身 Y 轴反转 pitch（消除板俯仰），
///   得到 armor2center 单位方向；旋转中心 = t + (r + thickness) * armor2center。
/// - 旋转轴方向 = armor2center 绕自身 Y 轴 -π/2。
/// - 目标板位置 = 旋转中心 + 绕 vertical 旋转 (-armor2center * r) + vertical * height_diff。
class OutpostSolution
{
public:
  struct Input {
    Eigen::Vector3d translation;        // 当前板平移
    Eigen::Quaterniond orientation;     // 当前板朝向（armor→参考系）
    OutpostArmorLevel source;
    OutpostArmorLevel target;
    double armor_thickness = 0;
  } input;

  struct Result {
    Eigen::Vector3d center;             // 旋转中心
    Eigen::Vector3d translation;        // 目标板平移
    Eigen::Quaterniond orientation;      // 目标板朝向
  } result;

  /// @brief 推算目标板位姿
  void solve() noexcept
  {
    const double radius = outpost_const::kRadius + input.armor_thickness;
    const double pitch = outpost_const::kPredictedPitch;
    const Eigen::Quaterniond & q = input.orientation;
    const Eigen::Vector3d & t = input.translation;

    // 板自身 X 轴方向（armor→center 半球内）
    const Eigen::Vector3d backward = q * Eigen::Vector3d::UnitX();
    // 绕板自身 Y 轴反转 pitch，得到水平指向旋转中心的单位方向
    const Eigen::Vector3d armor2center =
      Eigen::AngleAxisd(-pitch, q * Eigen::Vector3d::UnitY()) * backward;
    const Eigen::Vector3d rotation_center = t + radius * armor2center;

    // 旋转轴方向（垂直方向）
    const Eigen::Vector3d vertical =
      Eigen::AngleAxisd(-0.5 * outpost_const::kPi, q * Eigen::Vector3d::UnitY()) * armor2center;

    const auto [height_diff, rotate_angle] = get_transform(input.source, input.target);

    // 绕 vertical 旋转得到目标板位置
    const Eigen::AngleAxisd rotate2target(rotate_angle, vertical);
    const Eigen::Vector3d target_center = rotation_center +
      rotate2target * (-armor2center * radius) + vertical * height_diff;
    const Eigen::Vector3d toward = rotate2target * (q * Eigen::Vector3d::UnitX());

    const Eigen::Vector3d x_axis = toward.normalized();
    const Eigen::Vector3d y_axis = vertical.cross(x_axis).normalized();
    const Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();

    // 注意：Eigen 逗号初始化按行填充。rmcs 原版此处误用 `<< x_axis, y_axis, z_axis`，
    // 导致输出四元数实为 world→armor（转置），下游 q*UnitY 端点计算偏差约 6cm
    // （JS 真值对照确证：rows 版 0→1 位置偏离真值，cols 版 9 项几何验证全过）。
    // 正确做法：列 i = 板局部坐标轴 i 在参考系下的方向（armor→参考系）。
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;

    result.center = rotation_center;
    result.translation = target_center;
    result.orientation = Eigen::Quaterniond(rotation_matrix).normalized();
  }

private:
  /// @brief 源板到目标板的高度差（m）与旋转角（rad）
  static std::tuple<double, double> get_transform(
    OutpostArmorLevel from, OutpostArmorLevel to) noexcept
  {
    const double step = outpost_const::kHeightStep;
    const double turn = outpost_const::kAngleStep;
    if (from == to) return {0.0, 0.0};
    if (from == OutpostArmorLevel::UPPER && to == OutpostArmorLevel::MIDDLE)
      return {-1.0 * step, +turn};
    if (from == OutpostArmorLevel::MIDDLE && to == OutpostArmorLevel::LOWER)
      return {-1.0 * step, +turn};
    if (from == OutpostArmorLevel::LOWER && to == OutpostArmorLevel::UPPER)
      return {+2.0 * step, +turn};
    if (from == OutpostArmorLevel::UPPER && to == OutpostArmorLevel::LOWER)
      return {-2.0 * step, -turn};
    if (from == OutpostArmorLevel::MIDDLE && to == OutpostArmorLevel::UPPER)
      return {+1.0 * step, -turn};
    if (from == OutpostArmorLevel::LOWER && to == OutpostArmorLevel::MIDDLE)
      return {+1.0 * step, -turn};
    return {0.0, 0.0};
  }

public:
  /// @brief 由 in_right / is_upper 反查源板与目标板等级
  static std::tuple<OutpostArmorLevel, OutpostArmorLevel> get_level(
    bool is_right, bool is_upper) noexcept
  {
    if (is_right && is_upper) return {OutpostArmorLevel::LOWER, OutpostArmorLevel::UPPER};
    if (is_right && !is_upper) return {OutpostArmorLevel::UPPER, OutpostArmorLevel::MIDDLE};
    if (!is_right && is_upper) return {OutpostArmorLevel::MIDDLE, OutpostArmorLevel::UPPER};
    return {OutpostArmorLevel::UPPER, OutpostArmorLevel::LOWER};
  }
};

/// @brief 邻侧灯条 3D 端点推算
///
/// 移植自 rmcs `NeighborBarSolution`（utility/math/outpost.cpp L88-141）。
/// 给定当前板位姿与切板方向（in_right），推算邻侧上/下板灯条端点。
/// 结果含 4 组 Lightbar3d（upper_near/away, lower_near/away），6 点 PnP 仅取 near。
class NeighborBarSolution
{
public:
  struct Lightbar3d {
    Eigen::Vector3d upper;
    Eigen::Vector3d lower;
  };

  struct Input {
    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation;
    bool in_right = false;
    double armor_thickness = 0;
  } input;

  struct Result {
    Lightbar3d upper_near;
    Lightbar3d upper_away;
    Lightbar3d lower_near;
    Lightbar3d lower_away;
    Eigen::Vector3d center;
  } result;

  /// @brief 推算邻侧上/下板灯条端点
  void solve() noexcept
  {
    // 与 SP solver.cpp 中常量保持一致（小装甲宽度 / 灯条高度）
    constexpr double kSmallArmorWidth = 0.135;
    constexpr double kLightBarHeight = 0.056;
    const double y_step = 0.5 * kSmallArmorWidth;
    const double z_step = 0.5 * kLightBarHeight;

    OutpostSolution solution;
    solution.input.translation = input.translation;
    solution.input.orientation = input.orientation;
    solution.input.armor_thickness = input.armor_thickness;

    for (const bool is_upper : {true, false}) {
      const auto [source, target] = OutpostSolution::get_level(input.in_right, is_upper);
      solution.input.source = source;
      solution.input.target = target;
      solution.solve();

      const Eigen::Vector3d & center = solution.result.translation;
      const Eigen::Quaterniond & q = solution.result.orientation;
      const Eigen::Vector3d y_axis = q * Eigen::Vector3d::UnitY();
      const Eigen::Vector3d z_axis = q * Eigen::Vector3d::UnitZ();

      // 右灯条（+y 侧，"面对板"时位于观察者右侧）
      const Lightbar3d rside{
        center + y_axis * y_step + z_axis * z_step,  // upper
        center + y_axis * y_step - z_axis * z_step   // lower
      };
      // 左灯条（-y 侧）
      const Lightbar3d lside{
        center - y_axis * y_step + z_axis * z_step,
        center - y_axis * y_step - z_axis * z_step
      };

      // 取距当前板更近的作为 near
      const double lside_dist = (lside.upper - input.translation).norm();
      const double rside_dist = (rside.upper - input.translation).norm();
      const bool lside_closer = lside_dist < rside_dist;
      const Lightbar3d & near = lside_closer ? lside : rside;
      const Lightbar3d & away = lside_closer ? rside : lside;

      if (is_upper) {
        result.upper_near = near;
        result.upper_away = away;
      } else {
        result.lower_near = near;
        result.lower_away = away;
      }
      result.center = solution.result.center;
    }
  }
};

}  // namespace auto_aim

#endif  // AUTO_AIM__OUTPOST_GEOMETRY_HPP
