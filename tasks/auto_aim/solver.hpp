#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  /// @brief 前哨站 6 点 PnP 距离优化器
  ///
  /// 移植自 rmcs `OutpostDistanceOptimizer`。在 4 点 IPPE 基础上，
  /// 引入邻侧板灯条端点（由 NeighborBarSolution 几何推算 + 检测器 2D 端点）
  /// 作为额外约束，构造 6 点对应并热启动 cv::SOLVEPNP_ITERATIVE，
  /// 显著压制远距离 PnP 测距退化。
  ///
  /// @param armor 调用 solve() 后的 Armor（xyz/ypr 已被 IPPE 填充）
  /// @param is_right 切板方向是否为向右
  /// @param is_upper 当前观测是否为上灯条侧
  /// @param upper_pt 邻侧上灯条 2D 端点（上端）
  /// @param lower_pt 邻侧下灯条 2D 端点（下端）
  /// @param armor_thickness 装甲板厚度（m）
  void solve_outpost_6pt(
    Armor & armor, bool is_right, bool is_upper, const cv::Point2f & upper_pt,
    const cv::Point2f & lower_pt, double armor_thickness) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP