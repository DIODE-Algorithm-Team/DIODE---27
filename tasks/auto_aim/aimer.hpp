#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;       // 是否输出云台控制
  bool shootable = true;  // 预瞄边界点时为 false：云台继续跟踪，但 Shooter 不开火
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;

  // PR-5：前哨站动态攻击窗口 / 施密特滞回 / 边界点预瞄（移植 rmcs fire_control）
  double attack_window_;        // 攻击窗口全幅（rad）
  double window_hysteresis_;    // 滞回系数：enter=half*(1-hyst)，exit=half*(1+hyst)
  double degraded_angle_speed_;  // 角速度超过该值退化为不输出瞄准点（前哨站 0.8π 不触发）
  double window_redundancy_;    // 云台速度/加速度裕度系数
  double max_yaw_vel_;          // 云台最大 yaw 角速度（rad/s）
  double max_yaw_acc_;          // 云台最大 yaw 角加速度（rad/s²）
  bool attack_preaim_;          // 预瞄边界点时是否允许开火（默认 false：只跟踪不开火）
  int outpost_lock_id_ = -1;    // 施密特滞回锁定的板号（跨帧保持）

  AimPoint choose_aim_point(const Target & target);

  /// @brief 前哨站装甲板评估（移植 rmcs evaluate_armor）：
  /// 动态窗口夹紧 → 施密特滞回锁定 → 窗口内无板时返回来板一侧边界预瞄点
  AimPoint evaluate_outpost(const Target & target);

  /// @brief 锯齿波动态攻击窗口（移植 rmcs get_attack_window）
  /// @param omega 目标角速度绝对值；radius 旋转半径；distance 云台到旋转中心水平距离
  /// @param armor_num 板数（三板周期角 120°，四板 90°）
  /// @return 目标本体角度半幅；inf 不限制；NaN 退化
  double get_attack_window(double omega, double radius, double distance, int armor_num) const;

  /// @brief 来板一侧窗口边界在装甲板轨道上的预瞄点（移植 rmcs boundary_point）
  /// @param target 预测时刻目标；offset 带符号边界角（目标本体角度）
  static Eigen::Vector4d outpost_boundary_point(const Target & target, double offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP