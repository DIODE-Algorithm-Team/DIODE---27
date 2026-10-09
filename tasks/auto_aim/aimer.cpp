#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }

  // PR-5：前哨站开火控制参数（缺省值对齐 rmcs FireController::Config）
  attack_window_ = yaml["attack_window"].IsDefined() ? yaml["attack_window"].as<double>() / 57.3
                                                     : 40.0 / 57.3;
  window_hysteresis_ = yaml["window_hysteresis"].IsDefined()
                         ? yaml["window_hysteresis"].as<double>()
                         : 0.2;
  degraded_angle_speed_ = yaml["degraded_angle_speed"].IsDefined()
                            ? yaml["degraded_angle_speed"].as<double>()
                            : 12.0;
  window_redundancy_ =
    yaml["window_redundancy"].IsDefined() ? yaml["window_redundancy"].as<double>() : 0.8;
  max_yaw_vel_ = yaml["max_yaw_vel"].IsDefined() ? yaml["max_yaw_vel"].as<double>() : 10.0;
  max_yaw_acc_ = yaml["max_yaw_acc"].IsDefined() ? yaml["max_yaw_acc"].as<double>() : 200.0;
  attack_preaim_ =
    yaml["attack_preaim"].IsDefined() ? yaml["attack_preaim"].as<bool>() : false;
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  // PR-5：前哨站角速度退化防护（独立于 Target::predict 的 ±2.51 钳位，
  // 在 predict 之前触发，保证任何 omega>12 的前哨站都不会输出云台控制）
  if (target.name == ArmorName::outpost) {
    const double omega_abs = std::abs(target.ekf_x()[7]);
    if (omega_abs > degraded_angle_speed_) {
      outpost_lock_id_ = -1;
      debug_aim_point = {false, false, Eigen::Vector4d::Zero()};
      return {false, false, 0, 0};
    }
  }

  auto ekf = target.ekf();
  double delay_time =
    target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = 23;

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter]);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负

  // 预瞄边界点：云台照常跟踪，但清除 valid 以禁止 Shooter 开火
  // （Shooter 以 aimer.debug_aim_point.valid 作为开火条件之一）
  const bool shootable = debug_aim_point.shootable;
  if (!shootable) debug_aim_point.valid = false;

  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();

  // PR-5：前哨站走 rmcs 动态攻击窗口 + 施密特滞回 + 边界点预瞄
  if (target.name == ArmorName::outpost) return evaluate_outpost(target);

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped)
    return {true, true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (std::abs(target.ekf_x()[8]) <= 2) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, true, armor_xyza_list[id_list[0]]};
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > comming_angle_) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle_) return {true, true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle_) return {true, true, armor_xyza_list[i]};
  }

  return {false, false, armor_xyza_list[0]};
}

AimPoint Aimer::evaluate_outpost(const Target & target)
{
  const Eigen::VectorXd & ekf_x = target.ekf_x();
  const std::vector<Eigen::Vector4d> & aimpoints = target.armor_xyza_list();

  const double omega = ekf_x[7];
  const double omega_abs = std::abs(omega);

  // 退化接口：目标角速度过大时动态窗口失效（前哨站 ω≈0.8π 不会触发）
  if (omega_abs > degraded_angle_speed_) {
    outpost_lock_id_ = -1;
    tools::logger()->debug("[Aimer] Outpost attack window degraded: omega={:.2f}", omega);
    return {false, false, aimpoints[0]};
  }

  // 动态攻击窗口：锯齿波模型按云台速度/加速度约束给出半幅，再与配置窗口取 min
  const double radius = ekf_x[8];
  const double distance = std::hypot(ekf_x[0], ekf_x[2]);
  double half_window = attack_window_ / 2.0;
  const double dynamic_window =
    get_attack_window(omega_abs, radius, distance, static_cast<int>(aimpoints.size()));
  if (!std::isnan(dynamic_window) && !std::isinf(dynamic_window))
    half_window = std::min(half_window, dynamic_window);

  // 施密特滞回：进入阈值收紧、退出阈值放宽，避免边界板来回切换
  const double enter_window = half_window * (1.0 - window_hysteresis_);
  const double exit_window = half_window * (1.0 + window_hysteresis_);

  // 整车旋转中心的球坐标 yaw；delta 为装甲板相对弹道射线的本体角度
  const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
  std::vector<double> delta_angle_list;
  delta_angle_list.reserve(aimpoints.size());
  for (const auto & aimpoint : aimpoints) {
    delta_angle_list.emplace_back(tools::limit_rad(aimpoint[3] - center_yaw));
  }

  // 已锁定板只要未越出退出阈值就持续锁定
  if (outpost_lock_id_ >= 0 && outpost_lock_id_ < static_cast<int>(aimpoints.size())) {
    if (std::abs(delta_angle_list[outpost_lock_id_]) <= exit_window) {
      return {true, true, aimpoints[outpost_lock_id_]};
    }
  }

  // 在进入阈值内选择 |delta| 最小的板
  int best_id = -1;
  double best_abs = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(aimpoints.size()); ++i) {
    const double abs_delta = std::abs(delta_angle_list[i]);
    if (abs_delta <= enter_window && abs_delta < best_abs) {
      best_abs = abs_delta;
      best_id = i;
    }
  }

  if (best_id >= 0) {
    outpost_lock_id_ = best_id;
    return {true, true, aimpoints[best_id]};
  }

  // 窗口内无板：预瞄来板一侧的窗口边界，减少云台跟踪量
  // ω<0 时来板从 +half 一侧进入，ω>0 时从 -half 一侧进入（与 rmcs 一致）
  outpost_lock_id_ = -1;
  const double boundary = (omega < 0.0) ? +half_window : -half_window;
  // attack_preaim_=false 时只跟踪不开火；=true 时交由 Shooter 容差判断
  return {true, attack_preaim_, outpost_boundary_point(target, boundary)};
}

double Aimer::get_attack_window(
  double omega, double radius, double distance, int armor_num) const
{
  // 移植 rmcs FireController::get_attack_window，板间周期角由板数泛化
  // （四板 90°，三板 120°）。前哨站 0.8π 下动态半幅恒大于配置半幅，配置窗口生效。
  constexpr double kUnlimited = std::numeric_limits<double>::infinity();
  const double kDegraded = std::numeric_limits<double>::quiet_NaN();

  if (omega < 1e-1) return kUnlimited;
  if (omega > degraded_angle_speed_) return kDegraded;
  if (max_yaw_vel_ <= 0.0 || max_yaw_acc_ <= 0.0) return kUnlimited;
  if (radius <= 0.0 || distance <= 0.0) return kUnlimited;

  // 云台视角的装甲板轨道真实角跨度：板间半角 = π/n
  const double half_angle = CV_PI / armor_num;
  const double L = std::sqrt(
    distance * distance + radius * radius - 2.0 * distance * radius * std::cos(half_angle));
  const double stroke = 2.0 * std::asin(radius * std::sin(half_angle) / L);

  // 锯齿波参数：一块板扫过一个板间距的时间
  const double cycle_time = (2.0 * CV_PI / armor_num) / omega;
  const double v_track = stroke / cycle_time;

  const double v_max = max_yaw_vel_ * window_redundancy_;
  const double a_max = max_yaw_acc_ * window_redundancy_;

  // 预瞄时间：bang-bang（加速度主导）或梯形（速度饱和）
  double stable_window;
  if (std::sqrt(stroke * a_max) <= v_max + v_track) {
    stable_window = stroke - 2.0 * v_track * std::sqrt(stroke / a_max);
  } else {
    stable_window = v_max * stroke / (v_max + v_track) - v_track * (v_max + v_track) / a_max;
  }
  stable_window = std::max(stable_window, 0.0);

  // 云台视角半幅反解为目标本体角度半幅
  const double phi = stable_window * 0.5;
  const double sin_sum = (distance / radius) * std::sin(phi);
  if (sin_sum >= 1.0) return kUnlimited;
  return std::asin(sin_sum) - phi;
}

Eigen::Vector4d Aimer::outpost_boundary_point(const Target & target, double offset)
{
  // 移植 rmcs FireController::boundary_point
  const Eigen::VectorXd & ekf_x = target.ekf_x();
  const std::vector<Eigen::Vector4d> & aimpoints = target.armor_xyza_list();
  const double cx = ekf_x[0];
  const double cy = ekf_x[2];
  const double cz = ekf_x[4];

  // 来板：与边界同侧且 |delta| 最小的装甲板（delta 约定与 evaluate_outpost 相同）
  const double center_yaw = std::atan2(cy, cx);
  const Eigen::Vector4d * incoming = nullptr;
  double best_abs = std::numeric_limits<double>::infinity();
  for (const auto & aimpoint : aimpoints) {
    const double delta = tools::limit_rad(aimpoint[3] - center_yaw);
    if (((offset > 0.0) == (delta > 0.0)) && std::abs(delta) < best_abs) {
      best_abs = std::abs(delta);
      incoming = &aimpoint;
    }
  }

  const double radius =
    incoming ? std::hypot((*incoming)[0] - cx, (*incoming)[1] - cy) : ekf_x[8];
  const double z = incoming ? (*incoming)[2] : cz;
  // 中心→云台方向加边界角，即边界点在中心系中的方位
  const double angle = std::atan2(-cy, -cx) + offset;

  return {cx + radius * std::cos(angle), cy + radius * std::sin(angle), z, 0.0};
}

}  // namespace auto_aim