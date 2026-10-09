#ifndef AUTO_AIM__OUTPOST_HELPER_HPP
#define AUTO_AIM__OUTPOST_HELPER_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "outpost_const.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
/// @brief 前哨站板序号在线辨识缓冲区
///
/// 移植自 rmcs `OutpostModel::Impl::HeightBuffer`（outpost.cpp L98-175）。
///
/// 设计要点：
/// - 三槽位滑动缓冲，初值 NaN，`factor=0.1` 指数加权。
/// - `is_lower/is_upper` 在参与比较的任一值为 NaN 时返回 true，
///   使当前板 level 仅由已观测邻居确定。
/// - 仅观测两块板时也能给出已观测板间正确相对高度；
///   三块板齐后整组序号完全确定。
class HeightBuffer
{
public:
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  static constexpr double kFactor = 0.1;

  HeightBuffer() noexcept { data_.fill(kNaN); }

  /// @brief 切板后推进当前板索引
  /// @param sign 切板方向 +1/-1
  void go_next(int sign) noexcept
  {
    last_sign_ = sign;
    const int n = static_cast<int>(data_.size());
    int next = static_cast<int>(index_) + sign;
    next = ((next % n) + n) % n;  // 防止 size_t 与 int 混合下溢
    index_ = static_cast<std::size_t>(next);
  }

  /// @brief 用当前板高度观测做指数加权更新
  void update(double height) noexcept
  {
    auto & slot = data_[index_];
    slot = std::isnan(slot) ? height : slot * (1.0 - kFactor) + height * kFactor;
  }

  /// @brief 清理最旧板（三块板齐后纠正绝对序号时调用）
  void clean_oldest() noexcept
  {
    const int n = static_cast<int>(data_.size());
    int idx = static_cast<int>(index_) - 2 * last_sign_;
    idx = ((idx % n) + n) % n;  // 防止 size_t 与 int 混合下溢
    data_[static_cast<std::size_t>(idx)] = kNaN;
  }

  /// @brief 三槽位是否全部观测过
  bool all_observed() const noexcept
  {
    return !std::isnan(data_[0]) && !std::isnan(data_[1]) && !std::isnan(data_[2]);
  }

  /// @brief 当前板 level（0=高 / 1=中 / 2=低）
  int level() const noexcept { return level_at(index_); }

  /// @brief 指定槽位 level（0=高 / 1=中 / 2=低）
  int level_at(std::size_t pos) const noexcept
  {
    const auto n = data_.size();
    const auto height = data_[pos];
    const auto l_height = data_[(pos + n - 1) % n];
    const auto r_height = data_[(pos + 1) % n];

    // is_lower / is_upper 在遇到 NaN 时返回 true 的设计：
    // 当前板与未观测邻居的比较条件自动满足，不参与判定。
    // 因此当前板的 level 仅由已知邻居的真实比较确定。
    constexpr auto is_lower = [](double z, double other) {
      return (std::isnan(z) || std::isnan(other)) ? true : z < other;
    };
    constexpr auto is_upper = [](double z, double other) {
      return (std::isnan(z) || std::isnan(other)) ? true : z > other;
    };

    // 带 '*' 的即为观测到的装甲板
    /*--*/ if (is_upper(height, l_height) && is_upper(height, r_height)) {
      ///     [*]
      ///         [ ]
      /// [ ]
      return 0;  // 高
    } else if (is_lower(height, l_height) && is_upper(height, r_height)) {
      /// [ ]
      ///     [*]
      ///         [ ]
      return 1;  // 中
    } else if (is_lower(height, l_height) && is_lower(height, r_height)) {
      ///         [ ]
      /// [ ]
      ///     [*]
      return 2;  // 低
    }
    return 0;
  }

  /// @brief 读取某槽位当前滤波后的高度（NaN 表示未观测）
  double height_at(std::size_t pos) const noexcept { return data_[pos]; }

  /// @brief 当前板索引
  std::size_t index() const noexcept { return index_; }

private:
  std::array<double, 3> data_;
  int last_sign_ = 0;
  std::size_t index_ = 0;
};

/// @brief 切板方向连续两次同向确认状态机
///
/// 移植自 rmcs `OutpostModel::Impl`（outpost.cpp L189-194, L363-375）。
///
/// 抗视觉噪声：单帧 sign 不立即锁定，连续 kSignConfirmCount 次同向
/// 才将 rotation_sign 锁定。锁定前返回 0，锁定后返回 ±1。
struct PlateSwitchDetector
{
  int rotation_sign = 0;        // 已锁定方向 +1/-1，0=未确定
  int pending_sign = 0;         // 候选方向
  int sign_evidence_count = 0;  // 候选方向连续命中计数

  /// @brief 输入本次切板 sign，返回当前应使用的旋转方向
  /// @return 0=未锁定（用单帧 sign 或 0），±1=已锁定方向
  int update(int sign) noexcept
  {
    if (sign == 0) return rotation_sign;

    if (sign == pending_sign) {
      ++sign_evidence_count;
      if (sign_evidence_count >= outpost_const::kSignConfirmCount) {
        rotation_sign = sign;
      }
    } else {
      pending_sign = sign;
      sign_evidence_count = 1;
      // kSignConfirmCount==1 时立即锁定
      if (outpost_const::kSignConfirmCount <= 1) {
        rotation_sign = sign;
      }
    }
    return rotation_sign;
  }
};

/// @brief 邻板相对高度差（四象限表）
///
/// 移植自 rmcs `outpost_relative_height`（utility/math/outpost.cpp L38-48）。
/// 返回当前板相对邻侧板的高度差（step 倍数）。
///
/// | in_right | in_upper | delta_height |
/// |---|---|---|
/// | true  | true  | +2 step |
/// | true  | false | -1 step |
/// | false | true  | +1 step |
/// | false | false | -2 step |
inline double outpost_relative_height(bool in_right, bool in_upper) noexcept
{
  const auto step = outpost_const::kHeightStep;
  if (in_right && in_upper) return +2.0 * step;
  if (in_right && !in_upper) return -1.0 * step;
  if (!in_right && in_upper) return +1.0 * step;
  return -2.0 * step;
}

/// @brief 参考板坐标系折算：当前板观测折算回参考板
///
/// 移植自 rmcs `OutpostModel::Impl::correct()`（outpost.cpp L397-406）与
/// `outpost_relative_height`（utility/math/outpost.cpp L38-48）。
///
/// @param obs[in/out] 观测向量 [x, y, z, yaw]，原地折算
/// @param rel_see_index 当前板相对参考板偏移 -1/0/+1
/// @param in_right 切板方向是否为向右（+1）
/// @param in_upper 当前板是否为上灯条侧观测
inline void transform_to_reference(
  Eigen::Vector4d & obs, int rel_see_index, bool in_right, bool in_upper) noexcept
{
  if (rel_see_index == 0) return;

  const double delta_yaw = in_right ? outpost_const::kAngleStep : -outpost_const::kAngleStep;
  const double delta_height = outpost_relative_height(in_right, in_upper);

  obs(2) -= delta_height;                                  // 高度折算
  obs(3) = tools::limit_rad(obs(3) - delta_yaw);           // yaw 折算
}

/// @brief 观测位置门控：切板检测前的 xy_error 检查
///
/// 移植自 rmcs `OutpostModel::Impl::correct()`（outpost.cpp 切板检测前置检查）。
/// 距离 >5m 时阈值 0.20，否则 0.10，超限直接丢弃该帧（不更新 EKF）。
inline bool is_observation_valid(const Eigen::Vector4d & obs, const Eigen::Vector4d & last_obs) noexcept
{
  const double dx = obs(0) - last_obs(0);
  const double dy = obs(1) - last_obs(1);
  const double xy_error = std::sqrt(dx * dx + dy * dy);
  const double distance = std::sqrt(obs(0) * obs(0) + obs(1) * obs(1) + obs(2) * obs(2));
  const double threshold = (distance > 5.0) ? 0.20 : 0.10;
  return xy_error < threshold;
}

}  // namespace auto_aim

#endif  // AUTO_AIM__OUTPOST_HELPER_HPP
