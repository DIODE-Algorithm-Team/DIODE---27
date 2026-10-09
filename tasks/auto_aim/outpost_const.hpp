#ifndef AUTO_AIM__OUTPOST_CONST_HPP
#define AUTO_AIM__OUTPOST_CONST_HPP

#include <array>
#include <tuple>

namespace auto_aim::outpost_const
{
// 圆周率（避免引入 OpenCV / cmath 常量依赖）
constexpr double kPi = 3.14159265358979323846;

// 前哨站旋转半径（m），沿用 SP 旧值 0.2765
// rmcs 原值为 0.275，这里保持与 SP 现有 Target 构造一致
constexpr double kRadius = 0.2765;

// 前哨站装甲板厚度（m）
// rmcs: config.outpost_armor_thickness = 0.008
constexpr double kArmorThickness = 0.008;

// 三块装甲板的高度差（m）
// rmcs: kOutpostArmorHeightStep = 0.102
constexpr double kHeightStep = 0.102;

// 三块板 120° 夹角（rad）
constexpr double kAngleStep = 2.0 * kPi / 3.0;

// 前哨站装甲板预测 pitch（rad）
// rmcs: kPredictedOutpostArmorPitch = -15°
constexpr double kPredictedPitch = -15.0 * kPi / 180.0;

// 触发切板判定的最小 Yaw 角跳变（rad）
// 三块板夹角 120°，考虑观测局限，建议 50°~80° 之间
// rmcs 默认 50°
constexpr double kPlateSwitchYawMin = 50.0 * kPi / 180.0;

// 旋转方向锁定所需连续同向切板次数
// rmcs: kSignConfirmCount = 2，抗视觉噪声
constexpr int kSignConfirmCount = 2;

// 前哨站标称转速（rad/s），仅用于测试真值轨迹
// rmcs: kOutpostAngularSpeed = 0.8π
constexpr double kAngularSpeed = 0.8 * kPi;

// 三块板偏移表（按 高 → 中 → 低 排列）
// 每项 = {yaw_off, h_off}：
//   yaw_off 相对参考板的 yaw 偏移（rad）
//   h_off  相对参考板的高度偏移（step 数）
// 板 0 = 最高（参考板），板 1 = 中，板 2 = 低
constexpr std::array<std::tuple<double, double>, 3> kOffsetTable = {{
  {0.0 * kAngleStep, -0.0 * kHeightStep},
  {1.0 * kAngleStep, -1.0 * kHeightStep},
  {2.0 * kAngleStep, -2.0 * kHeightStep},
}};

}  // namespace auto_aim::outpost_const

#endif  // AUTO_AIM__OUTPOST_CONST_HPP
