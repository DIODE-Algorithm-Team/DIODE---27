#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim {
Tracker::Tracker(const std::string &config_path, Solver &solver)
    : solver_{solver}, detect_count_(0), temp_lost_count_(0), state_{"lost"},
      pre_state_{"lost"}, last_timestamp_(std::chrono::steady_clock::now()),
      omni_target_priority_{ArmorPriority::fifth} {
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red
                                                                  : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;

  // 图像光心从 camera_matrix 读取（替换 1440/2、1080/2 硬编码）
  if (yaml["camera_matrix"].IsDefined()) {
    const auto camera_matrix = yaml["camera_matrix"].as<std::vector<double>>();
    if (camera_matrix.size() >= 6) {
      image_cx_ = camera_matrix[2];
      image_cy_ = camera_matrix[5];
    }
  }
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(std::list<Armor> &armors,
                                 std::chrono::steady_clock::time_point t,
                                 bool use_enemy_color) {
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if(
      [&](const auto_aim::Armor &a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([this](const Armor &a, const Armor &b) {
    cv::Point2f img_center(static_cast<float>(image_cx_),
                           static_cast<float>(image_cy_));
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const auto_aim::Armor &a, const auto_aim::Armor &b) {
    return a.priority < b.priority;
  });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  const bool diverged = is_outpost_ ? outpost_target_->diverged()
                                    : static_cast<bool>(target_.diverged());
  if (state_ != "lost" && diverged) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  const auto &check_ekf = is_outpost_ ? outpost_target_->ekf() : target_.ekf();
  if (std::accumulate(check_ekf.recent_nis_failures.begin(),
                      check_ekf.recent_nis_failures.end(),
                      0) >= (0.4 * check_ekf.window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost")
    return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
    const std::vector<omniperception::DetectionResult> &detection_queue,
    std::list<Armor> &armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color) {
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([this](const Armor &a, const Armor &b) {
    cv::Point2f img_center(static_cast<float>(image_cx_),
                           static_cast<float>(image_cy_));
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
      [](const Armor &a, const Armor &b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() &&
           armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}",
                           ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !temp_target.armors.empty() &&
           temp_target.armors.front().priority < target_.priority &&
           target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
        temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  const bool diverged = is_outpost_ ? outpost_target_->diverged()
                                    : static_cast<bool>(target_.diverged());
  if (state_ != "lost" && diverged) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}}; // 返回switch_target和空的targets
  }

  if (state_ == "lost")
    return {switch_target, {}}; // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found) {
  if (state_ == "lost") {
    if (!found)
      return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_)
        state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found)
      return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200)
        state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        // 前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_)
        state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> &armors,
                         std::chrono::steady_clock::time_point t) {
  if (armors.empty())
    return false;

  auto &armor = armors.front();
  solver_.solve(armor);

  // 默认按普通目标处理；前哨站走独立 OutpostTarget
  is_outpost_ = false;
  outpost_target_.reset();

  // 根据兵种优化初始化参数
  auto is_balance =
      (armor.type == ArmorType::big) &&
      (armor.name == ArmorName::three || armor.name == ArmorName::four ||
       armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    // PR-4：前哨站改用 rmcs 5 维 EKF（中心 xyz + ω + 参考板 yaw）
    is_outpost_ = true;
    outpost_priority_ = armor.priority;
    outpost_target_.emplace(armor, t);
    refresh_outpost_snapshot();
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::update_target(std::list<Armor> &armors,
                            std::chrono::steady_clock::time_point t) {
  // ===== 前哨站路径：5 维 EKF，每帧只取最匹配的一块板（修复多板循环 update
  // 缺陷）=====
  if (is_outpost_) {
    outpost_target_->predict(t);

    Armor *best_armor = nullptr;
    double best_error = 1e10;
    const std::vector<Eigen::Vector4d> &xyza_list =
        outpost_target_->armor_xyza_list();

    for (auto &armor : armors) {
      if (armor.name != ArmorName::outpost || armor.type != ArmorType::small)
        continue;

      // solve 写 xyz/ypr/ypd 后再与三板预测位姿做匹配
      solver_.solve(armor);

      for (const auto &xyza : xyza_list) {
        Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
        // 与 Target::update 相同的匹配度量：板朝向差 + 方位角差
        auto angle_error =
            std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
            std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));
        if (angle_error < best_error) {
          best_error = angle_error;
          best_armor = &armor;
        }
      }
    }

    const bool found = best_armor != nullptr;
    if (found) {
      // PR-2 接线：6 点 PnP 距离优化（哨兵模式：邻侧灯条 2D 端点由 solver 内部 world2pixel 推算）
      const int sign = outpost_target_->rotation_sign();
      const bool is_right = (sign < 0);
      solver_.solve_outpost_6pt(
          *best_armor, is_right, /*is_upper=*/true,
          cv::Point2f(-1, -1), cv::Point2f(-1, -1),
          outpost_const::kArmorThickness);

      outpost_target_->update(*best_armor);
    }
    // 无论本帧是否观测到，都用 predict 后的状态刷新快照，
    // 保证 temp_lost 期间 Aimer 仍拿到最新外推位姿
    refresh_outpost_snapshot();
    return found;
  }

  target_.predict(t);

  int found_count = 0;
  double min_x = 1e10; // 画面最左侧
  for (const auto &armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type)
      continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }

  if (found_count == 0)
    return false;

  for (auto &armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type
        //  || armor.center.x != min_x
    )
      continue;

    solver_.solve(armor);

    target_.update(armor);
  }

  return true;
}

void Tracker::refresh_outpost_snapshot() {
  // 5 维状态 [cx, cy, cz, ω, ref_c2a_yaw] → 11 维 Target 状态
  //   [cx, vx, cy, vy, cz, v z, a, ω, r, l, h]
  // 11 维 Target 的 a 为 armor→center 方向 yaw（与 5 维 center→armor 差 π）；
  // 再减去 abs_ref·120°，使 Target 的 i 号板与 OutpostTarget 绝对板序（高/中/低）对齐。
  // cz 扣除参考板高度偏移，使 11 维 x[4] 表示最高板高度，三板高度台阶由
  // Target::h_armor_xyz 的 outpost 分支给出（PR-5，保证选中中/低板时俯仰正确）。
  const Eigen::VectorXd & s = outpost_target_->ekf_x();
  const auto & ref_offset = outpost_const::kOffsetTable[outpost_target_->abs_ref_index()];
  const double ref_yaw_off = std::get<0>(ref_offset);
  const double ref_h_off = std::get<1>(ref_offset);
  Eigen::VectorXd x11(11);
  x11 << s[0], 0.0, s[1], 0.0, s[2] - ref_h_off, 0.0,
    tools::limit_rad(s[4] + outpost_const::kPi - ref_yaw_off), s[3],
    outpost_const::kRadius, 0.0, 0.0;

  target_ = Target(x11, ArmorName::outpost, ArmorType::small, 3,
                   outpost_target_->convergened());
  target_.priority = outpost_priority_;
}

} // namespace auto_aim