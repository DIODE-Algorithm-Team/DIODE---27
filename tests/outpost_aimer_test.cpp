// PR-4/PR-5 链路集成测试：OutpostTarget 5 维 EKF → 11 维 Target 快照 → Aimer 开火控制
//
// 不依赖 OpenVINO / Solver：真值装甲板直接填充 xyz/ypr，快照映射复刻
// tracker.cpp::refresh_outpost_snapshot（Tracker 本体需 Solver，无法在此单测构造）。
//
// 验证项（对应 docs/outpost_migration.md 6.2 / 6.3 的可离线部分）：
//   1. 快照三板 xyz（含 PR-5 高度台阶）与 OutpostTarget::armor_xyza_list 一致
//   2. 首见板为中/低板（abs_ref≠0）时，快照三板绝对高度仍为 高/中/低
//   3. Aimer：每帧都输出云台控制；窗口内开火（debug_aim_point.valid），
//      窗口外预瞄（control=true 但 valid=false，shootable=false）
//   4. 开火点高度只可能是三板真实高度 {2.0, 1.898, 1.796}
//   5. 预瞄点位于半径轨道，且在来板一侧
//   6. 退化接口：|ω|>degraded_angle_speed 时不输出控制

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <list>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/core.hpp>

#include "../tasks/auto_aim/aimer.hpp"
#include "../tasks/auto_aim/armor.hpp"
#include "../tasks/auto_aim/outpost_const.hpp"
#include "../tasks/auto_aim/outpost_target.hpp"
#include "../tasks/auto_aim/target.hpp"
#include "../tools/math_tools.hpp"

using namespace auto_aim;

namespace {
constexpr double kPi = outpost_const::kPi;
constexpr double kRadius = outpost_const::kRadius;
constexpr double kPitch = outpost_const::kPredictedPitch;
constexpr double kTurn = outpost_const::kAngleStep;
constexpr double kDt = 0.01;
constexpr int kFrames = 600;

int passed = 0;
int failed = 0;

#define EXPECT_NEAR(a, b, eps)                                                 \
  do {                                                                         \
    if (std::abs((a) - (b)) >= (eps)) {                                        \
      std::printf("FAIL [%s:%d] expected %.6f got %.6f (eps=%.2e)\n",          \
                  __FILE__, __LINE__, static_cast<double>(b),                  \
                  static_cast<double>(a), static_cast<double>(eps));           \
      ++failed;                                                                \
    } else {                                                                   \
      ++passed;                                                                \
    }                                                                          \
  } while (0)

#define EXPECT_TRUE(cond)                                                      \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::printf("FAIL [%s:%d] %s\n", __FILE__, __LINE__, #cond);             \
      ++failed;                                                                \
    } else {                                                                   \
      ++passed;                                                                \
    }                                                                          \
  } while (0)

Armor make_armor(
  const Eigen::Vector3d & pos, double a2c_yaw, double yaw_noise, double pos_noise_len,
  std::mt19937 & rng)
{
  std::vector<cv::Point2f> kp(4);
  Armor armor(-1, 1.0f, cv::Rect(), kp);
  armor.name = ArmorName::outpost;
  armor.type = ArmorType::small;

  const Eigen::Vector3d dir = pos.normalized();
  std::normal_distribution<double> d_pos(0.0, pos_noise_len);
  armor.xyz_in_world = pos + dir * d_pos(rng);

  std::normal_distribution<double> d_yaw(0.0, yaw_noise);
  armor.ypr_in_world = {tools::limit_rad(a2c_yaw + d_yaw(rng)), kPitch, 0.0};
  return armor;
}

struct TrueFrame {
  std::array<Eigen::Vector3d, 3> pos;
  std::array<double, 3> a2c_yaw;
  int observed_id;
};

TrueFrame make_frame(double omega, int frame, double phase)
{
  const double angle = omega * (frame * kDt) + phase;
  TrueFrame f;
  f.observed_id = 0;
  for (int i = 0; i < 3; ++i) {
    const auto [yaw_off, h_off] = outpost_const::kOffsetTable[i];
    const double yaw = tools::limit_rad(angle + yaw_off);
    f.a2c_yaw[i] = yaw;
    f.pos[i] = Eigen::Vector3d{2.0 - kRadius * std::cos(yaw), -kRadius * std::sin(yaw),
                               2.0 + h_off};
  }
  for (int i = 1; i < 3; ++i)
    if (f.pos[i].x() < f.pos[f.observed_id].x()) f.observed_id = i;
  return f;
}

// 与 Tracker::refresh_outpost_snapshot 完全一致的 5 维 → 11 维快照映射
Target make_snapshot(const OutpostTarget & ot)
{
  const Eigen::VectorXd & s = ot.ekf_x();
  const auto & ref_offset = outpost_const::kOffsetTable[ot.abs_ref_index()];
  const double ref_yaw_off = std::get<0>(ref_offset);
  const double ref_h_off = std::get<1>(ref_offset);

  Eigen::VectorXd x11(11);
  x11 << s[0], 0.0, s[1], 0.0, s[2] - ref_h_off, 0.0,
    tools::limit_rad(s[4] + kPi - ref_yaw_off), s[3], kRadius, 0.0, 0.0;

  return Target(x11, ArmorName::outpost, ArmorType::small, 3, ot.convergened());
}

bool near_one_of(double v, const std::vector<double> & candidates, double eps)
{
  for (double c : candidates)
    if (std::abs(v - c) < eps) return true;
  return false;
}

struct ScenarioStat {
  int fire_count = 0;
  int preaim_count = 0;
  int no_control = 0;
  int max_snapshot_mismatch = 0;
};

ScenarioStat run_scenario(
  Aimer & aimer, double omega, double phase, unsigned seed, int expected_first_id,
  int expected_abs_ref)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> d_pos(0.0, 0.03);
  std::normal_distribution<double> d_yaw(0.0, 0.05);

  const auto t0 = std::chrono::steady_clock::now();
  const TrueFrame f1 = make_frame(omega, 1, phase);
  EXPECT_TRUE(f1.observed_id == expected_first_id);

  OutpostTarget ot(make_armor(f1.pos[f1.observed_id], f1.a2c_yaw[f1.observed_id], 0.0, 0.03, rng),
                   t0);

  ScenarioStat stat;
  const std::vector<double> kTrueHeights = {2.0, 2.0 - outpost_const::kHeightStep,
                                            2.0 - 2.0 * outpost_const::kHeightStep};

  for (int frame = 2; frame <= kFrames; ++frame) {
    const TrueFrame f = make_frame(omega, frame, phase);
    Armor armor = make_armor(f.pos[f.observed_id], f.a2c_yaw[f.observed_id], d_yaw(rng),
                             d_pos(rng), rng);
    ot.predict(kDt);
    ot.update(armor);

    Target snapshot = make_snapshot(ot);

    // 1) 快照三板 xyz 与 OutpostTarget 三板输出一致（含高度台阶）
    const auto l5 = ot.armor_xyza_list();
    const auto l11 = snapshot.armor_xyza_list();
    EXPECT_TRUE(l11.size() == 3 && l5.size() == 3);
    for (int i = 0; i < 3; ++i) {
      if (std::abs(l11[i][0] - l5[i][0]) > 0.02 || std::abs(l11[i][1] - l5[i][1]) > 0.02 ||
          std::abs(l11[i][2] - l5[i][2]) > 0.02) {
        ++stat.max_snapshot_mismatch;
      }
    }

    if (frame < 100) continue;  // 待 EKF 收敛后再评估 Aimer

    const auto timestamp = t0 + std::chrono::microseconds(frame * 10000);
    std::list<Target> targets = {snapshot};
    io::Command command = aimer.aim(targets, timestamp, 27.0, false);

    const AimPoint & ap = aimer.debug_aim_point;
    if (!command.control) {
      ++stat.no_control;
      continue;
    }
    if (ap.valid) {
      ++stat.fire_count;
      // 4) 开火点必须落在某块板的真实高度上
      EXPECT_TRUE(near_one_of(ap.xyza[2], kTrueHeights, 0.08));
      // yaw 朝向前哨站（中心在 (2,0)，含 yaw_offset 后仍应接近 0）
      EXPECT_TRUE(std::isfinite(command.yaw) && std::abs(command.yaw) < 0.15);
      EXPECT_TRUE(std::isfinite(command.pitch));
    } else {
      ++stat.preaim_count;
      // 3) 预瞄：可跟踪但不可开火
      EXPECT_TRUE(!ap.shootable);
      // 5) 预瞄点在半径轨道上
      const double rr = std::hypot(ap.xyza[0] - snapshot.ekf_x()[0],
                                   ap.xyza[1] - snapshot.ekf_x()[2]);
      EXPECT_NEAR(rr, kRadius, 0.02);
      EXPECT_TRUE(near_one_of(ap.xyza[2], kTrueHeights, 0.08));
      // 边界点方位：ω>0 时在 -half 一侧（y 偏小），ω<0 时在 +half 一侧
      const double local_y = ap.xyza[1] - snapshot.ekf_x()[2];
      if (omega > 0)
        EXPECT_TRUE(local_y >= -0.05);
      else
        EXPECT_TRUE(local_y <= 0.05);
    }
  }

  std::printf(
    "[omega=%+.2f phase=%.1f first=%d] abs_ref=%d fire=%d preaim=%d no_control=%d "
    "snapshot_mismatch=%d\n",
    omega, phase, expected_first_id, ot.abs_ref_index(), stat.fire_count, stat.preaim_count,
    stat.no_control, stat.max_snapshot_mismatch);

  EXPECT_TRUE(ot.abs_ref_index() == expected_abs_ref);
  EXPECT_TRUE(stat.max_snapshot_mismatch == 0);
  return stat;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string config_path = (argc >= 2) ? argv[1] : "configs/sentry.yaml";
  Aimer aimer(config_path);

  // 场景 A：首见最高板，正转
  ScenarioStat a = run_scenario(aimer, outpost_const::kAngularSpeed, 0.0, 42, 0, 0);
  // 场景 B：首见最低板（abs_ref=2），正转
  ScenarioStat b = run_scenario(aimer, outpost_const::kAngularSpeed, 2.0, 42, 2, 2);
  // 场景 C：首见中板（abs_ref=1），反转
  ScenarioStat c = run_scenario(aimer, -outpost_const::kAngularSpeed, 4.0, 7, 1, 1);

  for (const ScenarioStat * s : {&a, &b, &c}) {
    const int evaluated = s->fire_count + s->preaim_count;
    EXPECT_TRUE(evaluated > 0);
    // 收敛后不应失去控制（退化接口不触发）
    EXPECT_TRUE(s->no_control == 0);
    // 开火帧占比 20%~50%（半窗 20° / 板间距 120°，含滞回退出带）
    const double fire_ratio = static_cast<double>(s->fire_count) / evaluated;
    std::printf("fire_ratio=%.2f\n", fire_ratio);
    EXPECT_TRUE(fire_ratio > 0.20 && fire_ratio < 0.50);
    // 预瞄帧应占多数
    EXPECT_TRUE(s->preaim_count > s->fire_count);
  }

  // 6) 退化：|ω|=13 > degraded_angle_speed(12)，不输出云台控制
  {
    Eigen::VectorXd x11(11);
    x11 << 2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 13.0, kRadius, 0.0, 0.0;
    Target degraded(x11, ArmorName::outpost, ArmorType::small, 3, false);
    std::list<Target> targets = {degraded};
    io::Command command = aimer.aim(targets, std::chrono::steady_clock::now(), 27.0, false);
    EXPECT_TRUE(!command.control);
  }

  std::printf("\n==== outpost_aimer_test: %d passed, %d failed ====\n", passed, failed);
  return failed == 0 ? 0 : 1;
}
