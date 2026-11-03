#ifndef TOOLS__BALLISTICS_HPP
#define TOOLS__BALLISTICS_HPP

#include <Eigen/Dense>

namespace tools
{

/// @brief 弹道参数
///
/// 二次阻力模型：a = [-c·|v|·v ；-g]，其中 c 由 ρ·C_D·π·d²/(8m) 得到，单位 1/m。
/// 物理四项只是 c 的来源，实际以 resistance 为准（标定后直接填 resistance）。
struct BallisticParams
{
  double resistance = 0.0;      ///< 阻力合成系数 c（1/m），0 = 真空
  double gravity = 9.7833;      ///< 重力加速度（m/s²）
  double rk4_step = 0.002;      ///< RK4 步长（s）
  double max_fly_time = 3.0;    ///< 最大飞行时间（s）
  int max_iterations = 12;      ///< pitch 求解迭代上限
  double pitch_tolerance = 1e-4;  ///< pitch 收敛判据（rad）

  /// @brief 由物理四项计算 c（1/m）
  static double resistance_from(double mass, double diameter, double drag_coefficient,
                                double air_density);
};

/// @brief 求解结果
struct BallisticSolution
{
  bool solvable = false;
  double yaw = 0.0;       ///< rad，目标方位角（atan2(y, x)，与 Aimer 现有约定一致）
  double pitch = 0.0;     ///< rad，抬头为正（与 tools::Trajectory 一致）
  double fly_time = 0.0;  ///< s
};

/// @brief 求解云台角度与飞行时间
///
/// 目标点在**云台坐标系**（x 前、y 左、z 上），枪口视为旋转中心（muzzle_offset 暂未建模）。
/// 阻力与速度反向，水平方向不改变，因此 yaw 可解析求出；只剩 (pitch, fly_time) 两个未知量，
/// 用「给定 pitch 积分 → 水平距离匹配 → 残差为高度差」的 Newton 迭代求解。
///
/// @param v0 弹丸初速度（m/s），<=0 视为不可解
/// @param target 目标位置（m）
/// @param params 弹道参数
/// @return 不可解时 solvable=false，其余字段无效
BallisticSolution solve_ballistic(
  double v0, const Eigen::Vector3d & target, const BallisticParams & params);

}  // namespace tools

#endif  // TOOLS__BALLISTICS_HPP
