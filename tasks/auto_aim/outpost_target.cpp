#include "outpost_target.hpp"

#include <cmath>

#include "tools/math_tools.hpp"

namespace auto_aim
{

// ===== 噪声参数（移植自 rmcs OutpostModel::Config 默认值）=====
// 过程噪声 Q：前哨站中心在场地上绝对静止，xy 应尽可能小
constexpr double kProcessNoiseXY = 1e-6;     // rmcs: process_noise_xy
constexpr double kProcessNoiseZ = 1e-5;      // rmcs: process_noise_z
constexpr double kProcessNoiseSpeed = 1e-4;  // rmcs: process_noise_speed
constexpr double kProcessNoiseAngle = 1e-4;  // rmcs: process_noise_angle

// 观测噪声 R（单位：m / rad）。增大各值会加强滤波平滑，降低单帧视觉跳变敏感度
constexpr double kObservationNoiseXY = 0.005;  // rmcs: observation_noise_xy
constexpr double kObservationNoiseZ = 0.01;    // rmcs: observation_noise_z
constexpr double kObservationNoiseYaw = 0.01;  // rmcs: observation_noise_yaw

namespace
{
/// @brief 由 SP Armor 构造观测向量 [center.x, center.y, center.z, armor_yaw]
///
/// 换算与 rmcs correct() 逐行对齐：
///   backward = 板局部 X 轴（armor→center 方向，随板俯仰倾斜）
///   armor2center = backward 绕板自身 Y 轴反转 pitch → 水平指向旋转中心
///   armor_yaw = atan2(-armor2center.y, -armor2center.x)（center→armor 方向）
Eigen::Vector4d make_observation(const Armor & armor, double radius)
{
  const Eigen::Quaterniond q(tools::rotation_matrix(armor.ypr_in_world));
  const Eigen::Vector3d backward = q * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d armor2center =
    Eigen::AngleAxisd(-outpost_const::kPredictedPitch, q * Eigen::Vector3d::UnitY()) * backward;
  const Eigen::Vector3d center = armor.xyz_in_world + radius * armor2center;
  const double armor_yaw = std::atan2(-armor2center.y(), -armor2center.x());
  return {center.x(), center.y(), center.z(), armor_yaw};
}

}  // namespace

OutpostTarget::OutpostTarget(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  radius_(radius),
  armor_num_(armor_num),
  update_count_(0),
  abs_ref_index_(0),
  rel_see_index_(0),
  t_(t),
  t_last_update_(t)
{
  // 构造板即参考板：状态初始化为 [center, ω=0, ref_yaw]
  const Eigen::Vector4d obs = make_observation(armor, radius_);

  Eigen::VectorXd x0{{obs(0), obs(1), obs(2), 0.0, obs(3)}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[4] = tools::limit_rad(c[4]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);

  // 初始化 HeightBuffer：第一块板高度存入 data[0]
  height_buffer_.update(obs(2));

  // 初始化 last_observation（与 update() 末尾维护方式一致：
  // rel_see_index=0 时无折算补偿，直接取观测）
  last_observation_ = obs;
}

OutpostTarget::OutpostTarget(const Armor & armor, std::chrono::steady_clock::time_point t)
: OutpostTarget(armor, t, outpost_const::kRadius, 3, [] {
    Eigen::VectorXd diag{{1.0, 1.0, 1.0, 64.0, 0.4}};  // rmcs reset_covariance 默认值
    return diag;
  }())
{
}

void OutpostTarget::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void OutpostTarget::predict(double dt)
{
  // 状态转移雅可比：仅 yaw 对 ω 的耦合项 F(4,3)=dt，其余维不变
  // clang-format off
  Eigen::MatrixXd F{
    {1, 0, 0,  0, 0},
    {0, 1, 0,  0, 0},
    {0, 0, 1,  0, 0},
    {0, 0, 0,  1, 0},
    {0, 0, 0, dt, 1}
  };
  // clang-format on

  Eigen::VectorXd Q_dig{{kProcessNoiseXY, kProcessNoiseXY, kProcessNoiseZ, kProcessNoiseSpeed, kProcessNoiseAngle}};
  Eigen::MatrixXd Q = Q_dig.asDiagonal();

  // 状态转移：angle = normalize(angle + ω·dt)
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = x;
    x_prior[4] = tools::limit_rad(x_prior[4] + x_prior[3] * dt);
    return x_prior;
  };

  ekf_.predict(F, Q, f);
}

void OutpostTarget::update(const Armor & armor)
{
  // ===== 观测向量构造 =====
  Eigen::Vector4d observation = make_observation(armor, radius_);

  // ===== 观测位置门控：xy_error 超限判为噪声帧，直接丢弃（不更新 EKF）=====
  if (!is_observation_valid(observation, last_observation_)) return;

  // ===== 切板检测：|Δyaw| > kPlateSwitchYawMin 判为真实切板 =====
  {
    const double delta_yaw_obs = tools::limit_rad(observation(3) - last_observation_(3));

    if (std::abs(delta_yaw_obs) > outpost_const::kPlateSwitchYawMin) {
      // 从观测符号判断切板方向：yaw 增大 = 向右切
      const int sign = delta_yaw_obs > 0.0 ? +1 : -1;

      // 旋转方向确认：连续 kSignConfirmCount 次同方向切板后锁定；
      // 未锁定时用单帧 sign 兜底（rmcs: go_sign = rotation_sign ?: sign）
      const int locked_sign = plate_switch_detector_.update(sign);
      const int go_sign = locked_sign != 0 ? locked_sign : sign;

      // 更新 HeightBuffer：推进当前板槽位并写入本次高度观测
      height_buffer_.go_next(go_sign);
      height_buffer_.update(observation(2));

      // 当前板相对参考板位置 -1/0/+1
      rel_see_index_ += sign;
      if (rel_see_index_ > +1) rel_see_index_ -= 3;
      if (rel_see_index_ < -1) rel_see_index_ += 3;

      // 三块板齐后纠正参考板绝对序号，并清理最旧槽位
      if (height_buffer_.all_observed()) {
        abs_ref_index_ = (height_buffer_.level() - rel_see_index_ + 3) % 3;
        height_buffer_.clean_oldest();
      }
    }
  }

  // ===== 参考板坐标系折算：当前板观测折算回参考板 =====
  double delta_yaw = 0.0;
  double delta_height = 0.0;
  if (rel_see_index_ != 0) {
    const bool in_right = rel_see_index_ > 0;
    const bool in_upper = observation(2) > ekf_.x[2];  // 当前板是否高于参考板
    delta_yaw = in_right ? +outpost_const::kAngleStep : -outpost_const::kAngleStep;
    delta_height = outpost_relative_height(in_right, in_upper);
  }
  observation(2) -= delta_height;
  observation(3) = tools::limit_rad(observation(3) - delta_yaw);

  // ===== EKF 观测更新（H 线性：取 x,y,z,yaw）=====
  // clang-format off
  Eigen::MatrixXd H{
    {1, 0, 0, 0, 0},
    {0, 1, 0, 0, 0},
    {0, 0, 1, 0, 0},
    {0, 0, 0, 0, 1}
  };
  // clang-format on

  Eigen::VectorXd R_dig{{kObservationNoiseXY, kObservationNoiseXY, kObservationNoiseZ, kObservationNoiseYaw}};
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  ekf_.update(observation, H, R, z_subtract);

  // ===== last_observation 维护 =====
  // 用 EKF 后验 state（低噪声平滑值）反补偿回当前板坐标系，
  // 使下一帧切板检测仍以当前板为基准
  last_observation_ << ekf_.x[0], ekf_.x[1], ekf_.x[2] + delta_height,
    tools::limit_rad(ekf_.x[4] + delta_yaw);

  update_count_++;
  t_last_update_ = std::chrono::steady_clock::now();
}

Eigen::VectorXd OutpostTarget::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & OutpostTarget::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> OutpostTarget::armor_xyza_list() const
{
  // 三块板按 高→中→低 排列（kOffsetTable 序）。EKF state 跟踪参考板，
  // 各板 = 旋转中心 + radius * (state[4] + 板间 yaw 偏移) 方向 + 板间高度偏移
  std::vector<Eigen::Vector4d> list;
  list.reserve(armor_num_);

  for (int i = 0; i < armor_num_; i++) {
    const auto [yaw_off_i, h_off_i] = outpost_const::kOffsetTable[i];
    const auto [yaw_off_ref, h_off_ref] = outpost_const::kOffsetTable[abs_ref_index_];

    // center→armor 方向 yaw（state[4] 约定）
    const double center2armor_yaw = tools::limit_rad(ekf_.x[4] + (yaw_off_i - yaw_off_ref));
    const double delta_height = h_off_i - h_off_ref;

    const double armor_x = ekf_.x[0] + radius_ * std::cos(center2armor_yaw);
    const double armor_y = ekf_.x[1] + radius_ * std::sin(center2armor_yaw);
    const double armor_z = ekf_.x[2] + delta_height;

    // SP 约定：a = armor→center 方向 yaw（与 Armor.ypr_in_world[0] 可直接比较）
    const double a = tools::limit_rad(center2armor_yaw + outpost_const::kPi);
    list.push_back({armor_x, armor_y, armor_z, a});
  }
  return list;
}

bool OutpostTarget::diverged() const
{
  const Eigen::VectorXd & x = ekf_.x;
  const Eigen::MatrixXd & cov = ekf_.P;

  // 超时 500ms 未成功更新
  if (tools::delta_time(std::chrono::steady_clock::now(), t_last_update_) > 0.5) return true;

  if (cov(0, 0) > 150.0 || cov(1, 1) > 150.0) return true;         // 协方差发散
  if (std::abs(x[0]) > 15.0 || std::abs(x[1]) > 15.0) return true;  // 中心位置超界
  if (std::abs(x[3]) > 10.0 * outpost_const::kPi) return true;      // 转速超界
  if (x.hasNaN() || !x.allFinite() || !cov.allFinite()) return true;

  return false;
}

bool OutpostTarget::convergened() const
{
  // rmcs: update_count >= 5 且 center xy / yaw 协方差足够小
  if (update_count_ < 5) return false;

  const Eigen::MatrixXd & cov = ekf_.P;
  return cov(0, 0) < 0.01 && cov(1, 1) < 0.01 && cov(4, 4) < 0.01;
}

}  // namespace auto_aim
