#ifndef AUTO_AIM__OUTPOST_TARGET_HPP
#define AUTO_AIM__OUTPOST_TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include "armor.hpp"
#include "outpost_const.hpp"
#include "outpost_helper.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

/// @brief 前哨站专用目标类（独立于 11 维 Target，零侵入）
///
/// 移植自 rmcs `OutpostModel`（module/tracker/model/outpost.cpp）。
///
/// 状态量（5 维）：
///   x[0..2]  旋转中心（参考板高度处的虚拟中心点）世界系坐标 x / y / z
///   x[3]     转速 ω (rad/s)
///   x[4]     参考板 center→armor 方向 yaw (rad)
///
/// 观测量（4 维）：[center.x, center.y, center.z, armor_yaw]
/// 由 SP `Armor`（xyz_in_world + ypr_in_world）内部换算得到，约定：
///   - center = 板中心 + radius * armor2center（水平指向旋转中心的方向），
///     处于当前板高度处；切板后折算回参考板高度。
///   - armor_yaw = center→armor 方向 yaw（rmcs 约定，与 SP ypr[0] 的
///     armor→center 方向相反，仅在类内部使用）。
///
/// 接入的关键技术点（对应 docs/outpost_migration.md 阶段 3）：
///   - HeightBuffer 板序号在线辨识（outpost_helper.hpp）
///   - PlateSwitchDetector 切板方向连续两次同向锁定
///   - transform_to_reference 参考板坐标系折算
///   - 观测位置门控（xy_error 超限丢帧）
class OutpostTarget
{
public:
  ArmorName name;

  /// @brief 以第一块观测板为参考板构造（rmcs 默认 P0）
  OutpostTarget(const Armor & armor, std::chrono::steady_clock::time_point t);

  /// @brief 对齐 SP `Target` 签名风格（radius/armor_num/P0_dig 可由 Tracker 配置传入）
  OutpostTarget(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;

  /// @brief 三块板 [x, y, z, a] 列表，按 高→中→低 排列（对齐 rmcs full()）
  ///        a 为 armor→center 方向 yaw，可与 SP Armor.ypr_in_world[0] 直接比较
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;
  bool convergened() const;

  // 调试 / 测试用只读接口
  int abs_ref_index() const { return abs_ref_index_; }  // 参考板绝对序号 0/1/2
  int rel_see_index() const { return rel_see_index_; }  // 当前板相对参考板偏移 -1/0/+1
  int rotation_sign() const { return plate_switch_detector_.rotation_sign; }

private:
  double radius_;
  int armor_num_;
  std::size_t update_count_;

  int abs_ref_index_;  // 参考板绝对序号 0/1/2（第一块观测板，三块板齐后纠正）
  int rel_see_index_;  // 当前板相对参考板位置 -1/0/+1

  HeightBuffer height_buffer_;
  PlateSwitchDetector plate_switch_detector_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;                // 上次 predict 时间
  std::chrono::steady_clock::time_point t_last_update_;    // 上次成功 update 时间
  Eigen::Vector4d last_observation_;                       // 上一合法帧观测（当前板坐标系）
};

}  // namespace auto_aim

#endif  // AUTO_AIM__OUTPOST_TARGET_HPP
