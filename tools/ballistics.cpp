#include "ballistics.hpp"

#include <algorithm>
#include <cmath>

#include "trajectory.hpp"

namespace tools
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

/// @brief 一次积分的结果
struct Eval
{
  bool reached = false;  ///< 是否在最大飞行时间内到达目标水平距离
  double z = 0.0;        ///< 到达水平距离处的高度（m）
  double t = 0.0;        ///< 飞行时间（s）
};

Eigen::Vector3d acceleration(
  const Eigen::Vector3d & velocity, double resistance, double gravity)
{
  const double speed = velocity.norm();
  const double k = resistance * speed;
  return {-k * velocity.x(), -k * velocity.y(), -gravity - k * velocity.z()};
}

/// @brief 给定 pitch 积分，返回到达水平距离 d 处的高度与飞行时间
Eval integrate(double pitch, double yaw, double v0, double d, const BallisticParams & p)
{
  Eval out;

  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel{v0 * cp * cy, v0 * cp * sy, v0 * sp};

  const double h = p.rk4_step;
  double t = 0.0;

  while (t < p.max_fly_time) {
    const double xy_before = std::hypot(pos.x(), pos.y());
    if (xy_before >= d) {
      out.reached = true;
      out.z = pos.z();
      out.t = t;
      return out;
    }

    const Eigen::Vector3d pos_before = pos;
    const double t_before = t;

    // RK4
    const Eigen::Vector3d k1p = vel;
    const Eigen::Vector3d k1v = acceleration(vel, p.resistance, p.gravity);

    const Eigen::Vector3d v2 = vel + 0.5 * h * k1v;
    const Eigen::Vector3d k2p = v2;
    const Eigen::Vector3d k2v = acceleration(v2, p.resistance, p.gravity);

    const Eigen::Vector3d v3 = vel + 0.5 * h * k2v;
    const Eigen::Vector3d k3p = v3;
    const Eigen::Vector3d k3v = acceleration(v3, p.resistance, p.gravity);

    const Eigen::Vector3d v4 = vel + h * k3v;
    const Eigen::Vector3d k4p = v4;
    const Eigen::Vector3d k4v = acceleration(v4, p.resistance, p.gravity);

    pos += h / 6.0 * (k1p + 2.0 * k2p + 2.0 * k3p + k4p);
    vel += h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
    t += h;

    const double xy_after = std::hypot(pos.x(), pos.y());
    if (xy_after >= d) {
      const double denom = xy_after - xy_before;
      const double ratio = denom > 1e-12 ? (d - xy_before) / denom : 1.0;
      out.reached = true;
      out.z = pos_before.z() + ratio * (pos.z() - pos_before.z());
      out.t = t_before + ratio * (t - t_before);
      return out;
    }
  }

  return out;
}

}  // namespace

double BallisticParams::resistance_from(
  double mass, double diameter, double drag_coefficient, double air_density)
{
  if (mass <= 0.0) return 0.0;
  return air_density * drag_coefficient * kPi * diameter * diameter / (8.0 * mass);
}

BallisticSolution solve_ballistic(
  double v0, const Eigen::Vector3d & target, const BallisticParams & params)
{
  BallisticSolution result;

  const double d = std::hypot(target.x(), target.y());
  if (v0 <= 1e-3) return result;

  result.yaw = std::atan2(target.y(), target.x());

  // 极小水平距离：直接抬头对准（避免除零），飞行时间取 0
  if (d < 1e-3) {
    result.solvable = true;
    result.pitch = std::atan2(target.z(), std::max(1e-3, d));
    result.fly_time = 0.0;
    return result;
  }

  // 真空解析解作为初值（同时给出「物理上能否打到」的初判）
  tools::Trajectory vacuum(v0, d, target.z());
  if (vacuum.unsolvable) return result;

  const auto residual = [&](double pitch) -> double {
    const Eval e = integrate(pitch, result.yaw, v0, d, params);
    // 没到达目标水平距离：说明 pitch 过高（射程不够），给一个大残差把迭代往下拉
    if (!e.reached) return 1e3;
    return e.z - target.z();
  };

  double pitch = vacuum.pitch;
  constexpr double kMaxPitch = 1.3;   // ~75°
  constexpr double kMinPitch = -0.35; // ~-20°
  constexpr double kStepDeg = 1e-3;   // 数值微分步长（rad）

  // 低角度种子可能「打不到」（射程不足），逐步抬高到可达区间再迭代
  if (!integrate(pitch, result.yaw, v0, d, params).reached) {
    bool lifted = false;
    for (double candidate : {0.6, 0.8, 1.0, 1.15, 1.3}) {
      if (integrate(candidate, result.yaw, v0, d, params).reached) {
        pitch = candidate;
        lifted = true;
        break;
      }
    }
    if (!lifted) return result;  // 任何角度都打不到：确实不可解
  }

  bool converged = false;
  for (int i = 0; i < params.max_iterations; ++i) {
    const double r = residual(pitch);
    if (std::abs(r) < 1e-3) {
      converged = true;
      break;
    }

    const double r2 = residual(pitch + kStepDeg);
    const double slope = (r2 - r) / kStepDeg;
    if (!std::isfinite(slope) || std::abs(slope) < 1e-6) break;

    const double delta = std::clamp(r / slope, -0.2, 0.2);
    pitch = std::clamp(pitch - delta, kMinPitch, kMaxPitch);

    if (std::abs(delta) < params.pitch_tolerance) {
      converged = true;
      break;
    }
  }

  // Newton 失败时的兜底：在 [-20°, 75°] 上扫描找变号区间，再二分
  if (!converged) {
    double lo = kMinPitch;
    double hi = kMaxPitch;
    double f_lo = residual(lo);
    double f_hi = residual(hi);

    if (f_lo > 0.0 && f_hi > 0.0) return result;  // 全程偏高：打不到
    if (f_lo < 0.0 && f_hi < 0.0) return result;  // 全程偏低：初速不足

    if (f_lo > f_hi) {
      std::swap(lo, hi);
      std::swap(f_lo, f_hi);
    }

    for (int i = 0; i < 40; ++i) {
      const double mid = 0.5 * (lo + hi);
      const double f_mid = residual(mid);
      if (std::abs(f_mid) < 1e-3) {
        pitch = mid;
        converged = true;
        break;
      }
      if ((f_mid > 0.0) == (f_lo > 0.0)) {
        lo = mid;
        f_lo = f_mid;
      } else {
        hi = mid;
        f_hi = f_mid;
      }
    }
    if (!converged) return result;
  }

  const Eval final_eval = integrate(pitch, result.yaw, v0, d, params);
  if (!final_eval.reached) return result;

  result.solvable = true;
  result.pitch = pitch;
  result.fly_time = final_eval.t;
  return result;
}

}  // namespace tools
