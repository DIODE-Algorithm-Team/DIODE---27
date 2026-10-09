#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "outpost_geometry.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

//solvePnP（获得姿态）
void Solver::solve(Armor & armor) const
{
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  optimize_yaw(armor);
}

// 前哨站 6 点 PnP 距离优化器（移植自 rmcs `OutpostDistanceOptimizer`）
// 几何：4 个装甲板角点 + 2 个邻侧灯条端点（由 NeighborBarSolution 推算），
//       用 4 点 IPPE 结果热启动 SOLVEPNP_ITERATIVE，压制远距离测距退化。
void Solver::solve_outpost_6pt(
  Armor & armor, bool is_right, bool is_upper, const cv::Point2f & upper_pt,
  const cv::Point2f & lower_pt, double armor_thickness) const
{
  // 前哨站恒为小装甲板
  const auto & object_points_4pt = SMALL_ARMOR_POINTS;

  // 1. 4 点 IPPE 获取相机系下初始 rvec/tvec
  cv::Vec3d initial_rvec, initial_tvec;
  cv::solvePnP(
    object_points_4pt, armor.points, camera_matrix_, distort_coeffs_, initial_rvec, initial_tvec,
    false, cv::SOLVEPNP_IPPE);
  if (initial_tvec[2] <= 0.0) return;  // 前向约束

  // 2. 将 IPPE 结果变换到世界系，供 NeighborBarSolution 推算邻侧板位姿
  cv::Mat rmat;
  cv::Rodrigues(initial_rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Vector3d t_armor_in_camera;
  cv::cv2eigen(initial_tvec, t_armor_in_camera);

  Eigen::Vector3d xyz_in_world =
    R_gimbal2world_ * (R_camera2gimbal_ * t_armor_in_camera + t_camera2gimbal_);
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_camera2gimbal_ * R_armor2camera;
  Eigen::Quaterniond q_armor2world = Eigen::Quaterniond(R_armor2world).normalized();

  // 3. 推算邻侧板灯条 3D 端点（世界系）
  NeighborBarSolution neighbor;
  neighbor.input.translation = xyz_in_world;
  neighbor.input.orientation = q_armor2world;
  neighbor.input.in_right = is_right;
  neighbor.input.armor_thickness = armor_thickness;
  neighbor.solve();

  // 由 is_upper 选定上灯条侧或下灯条侧
  const auto & selected = is_upper ? neighbor.result.upper_near : neighbor.result.lower_near;
  const Eigen::Vector3d upper_world = selected.upper;
  const Eigen::Vector3d lower_world = selected.lower;

  // 哨兵模式：若调用者未提供有效邻侧灯条 2D 端点（x<0），
  // 用 world2pixel 把推算出的世界系端点投影到图像（纯几何闭环，无额外观测）
  cv::Point2f upper_pt_use = upper_pt;
  cv::Point2f lower_pt_use = lower_pt;
  if (upper_pt_use.x < 0 || lower_pt_use.x < 0) {
    std::vector<cv::Point3f> world_pts;
    world_pts.emplace_back((float)upper_world.x(), (float)upper_world.y(), (float)upper_world.z());
    world_pts.emplace_back((float)lower_world.x(), (float)lower_world.y(), (float)lower_world.z());
    auto pixel_pts = world2pixel(world_pts);
    if (pixel_pts.size() >= 2) {
      upper_pt_use = pixel_pts[0];
      lower_pt_use = pixel_pts[1];
    } else {
      return;
    }
  }

  // 4. 邻侧灯条端点变换到当前板局部系（与 SMALL_ARMOR_POINTS 同参考系）
  const Eigen::Matrix3d R_world2armor = R_armor2world.transpose();
  const Eigen::Vector3d upper_local = R_world2armor * (upper_world - xyz_in_world);
  const Eigen::Vector3d lower_local = R_world2armor * (lower_world - xyz_in_world);

  // 5. 构造 6 点对应（4 板角点 + 2 邻侧灯条端点）
  std::vector<cv::Point3f> object_points_6pt;
  object_points_6pt.reserve(6);
  for (const auto & p : SMALL_ARMOR_POINTS) object_points_6pt.push_back(p);
  object_points_6pt.emplace_back(
    static_cast<float>(upper_local.x()), static_cast<float>(upper_local.y()),
    static_cast<float>(upper_local.z()));
  object_points_6pt.emplace_back(
    static_cast<float>(lower_local.x()), static_cast<float>(lower_local.y()),
    static_cast<float>(lower_local.z()));

  std::vector<cv::Point2f> image_points_6pt;
  image_points_6pt.reserve(6);
  for (const auto & p : armor.points) image_points_6pt.push_back(p);
  image_points_6pt.push_back(upper_pt_use);
  image_points_6pt.push_back(lower_pt_use);

  // 6. 6 点 iterative PnP（热启动）
  cv::Vec3d optimized_rvec = initial_rvec;
  cv::Vec3d optimized_tvec = initial_tvec;
  const bool solved = cv::solvePnP(
    object_points_6pt, image_points_6pt, camera_matrix_, distort_coeffs_, optimized_rvec,
    optimized_tvec, true, cv::SOLVEPNP_ITERATIVE);
  if (!solved || optimized_tvec[2] <= 0.0) return;

  // 7. 把优化结果写回 armor（世界系）
  cv::Mat opt_rmat;
  cv::Rodrigues(optimized_rvec, opt_rmat);
  Eigen::Matrix3d R_armor2camera_opt;
  cv::cv2eigen(opt_rmat, R_armor2camera_opt);
  Eigen::Vector3d t_armor_in_camera_opt;
  cv::cv2eigen(optimized_tvec, t_armor_in_camera_opt);

  armor.xyz_in_gimbal = R_camera2gimbal_ * t_armor_in_camera_opt + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
  Eigen::Matrix3d R_armor2gimbal_opt = R_camera2gimbal_ * R_armor2camera_opt;
  Eigen::Matrix3d R_armor2world_opt = R_gimbal2world_ * R_armor2gimbal_opt;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal_opt, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world_opt, 2, 1, 0);
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 8. 基于更新后的 xyz 重新优化 yaw
  optimize_yaw(armor);
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // solve
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

void Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  // 三点二次插值精化（移植自 rmcs `RobustPnpSolution::solve` L246-263）：
  // 在最优 yaw ±1° 处再各采样一次重投影误差，用二次函数拟合极值点，
  // 可得亚度级精度。仅当插值点误差更小时才接受，避免噪声误导。
  if (min_error < 1e10 - 1.0) {
    constexpr double kStepDeg = 1.0;
    const double step = kStepDeg * CV_PI / 180.0;
    const double err_low = armor_reprojection_error(armor, tools::limit_rad(best_yaw - step), 0.0);
    const double err_up = armor_reprojection_error(armor, tools::limit_rad(best_yaw + step), 0.0);

    if (err_low < 1e10 - 1.0 && err_up < 1e10 - 1.0) {
      const double denom = err_low - 2.0 * min_error + err_up + 1e-9;
      if (std::abs(denom) > 1e-12) {
        const double interpolated_yaw = best_yaw + step * 0.5 * (err_low - err_up) / denom;
        const double interpolated_error = armor_reprojection_error(armor, interpolated_yaw, 0.0);
        if (interpolated_error < min_error) {
          best_yaw = interpolated_yaw;
          min_error = interpolated_error;
        }
      }
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints) const
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim