#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>
#include <string>

#include "armor.hpp"
#include "outpost_target.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  // PR-4：前哨站走独立 5 维 EKF；target_ 每帧刷新为其 11 维快照，
  // 供 Aimer / 优先级比较 / 状态机复用，调用方无感知
  bool is_outpost_ = false;
  std::optional<OutpostTarget> outpost_target_;
  ArmorPriority outpost_priority_ = ArmorPriority::fifth;
  double image_cx_ = 720.0;  // 图像中心（px），从 camera_matrix 读取，替换硬编码
  double image_cy_ = 540.0;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  /// @brief 用 OutpostTarget 当前状态刷新 target_ 快照（含 predict 后的时刻）
  void refresh_outpost_snapshot();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP