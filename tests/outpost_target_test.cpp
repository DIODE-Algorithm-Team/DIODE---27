// OutpostTarget 5 维 EKF 集成测试
// 对齐 rmcs tool/cxx/test_outpost_ekf.cpp 的真值轨迹构造（去除 ROS marker）：
//   - 真值中心 (2, 0, 2)，ω=±0.8π，三块板绕中心旋转
//   - 可见板 = 距相机（原点）最近的板（argmin position.x）
//   - 位置噪声 σ=0.03m（沿视线方向），yaw 噪声 σ=0.05rad
//
// 验证项（对应 docs/outpost_migration.md 6.2）：
//   1. IT_outpost_ekf_converge：EKF 收敛，中心误差 <0.05m，ω 误差 <0.1rad/s，
//      参考板 yaw 误差 <0.05rad
//   2. IT_outpost_switch_plate：切板方向锁定（正转 +1 / 反转 -1），
//      切板后 rel_see_index 正确折叠到 {-1,0,+1}
//   3. IT_outpost_abs_ref_index：三块板齐后 abs_ref_index == 0（首板为最高板）
//   4. armor_xyza_list：三板水平半径恒为 r，当前可见板位姿与真值一致
//   5. 观测位置门控：xy 跳变 0.5m 的噪声帧被丢弃，EKF 状态不变

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include <opencv2/core.hpp>

#include "../tasks/auto_aim/armor.hpp"
#include "../tasks/auto_aim/outpost_const.hpp"
#include "../tasks/auto_aim/outpost_target.hpp"
#include "../tools/math_tools.hpp"

using namespace auto_aim;

namespace {
constexpr double kPi = outpost_const::kPi;
constexpr double kRadius = outpost_const::kRadius;
constexpr double kPitch = outpost_const::kPredictedPitch;
constexpr double kStep = outpost_const::kHeightStep;
constexpr double kTurn = outpost_const::kAngleStep;
constexpr double kDt = 0.01;
constexpr int kFrames = 600; // 6s，覆盖 ≥6 次切板

constexpr double kPosSigma = 0.03;
constexpr double kYawSigma = 0.05;

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

/// 由真值位姿构造 SP Armor（xyz + ypr），叠加噪声
Armor make_armor(const Eigen::Vector3d &pos, double a2c_yaw, double yaw_noise,
                 double pos_noise_len, std::mt19937 &rng) {
  std::vector<cv::Point2f> kp(4); // 占位关键点，PnP 不参与本测试
  Armor armor(-1, 1.0f, cv::Rect(), kp);
  armor.name = ArmorName::outpost;
  armor.type = ArmorType::small;

  // 位置噪声沿视线方向（与 rmcs test 一致）
  const Eigen::Vector3d dir = pos.normalized();
  std::normal_distribution<double> d_pos(0.0, pos_noise_len);
  armor.xyz_in_world = pos + dir * d_pos(rng);

  std::normal_distribution<double> d_yaw(0.0, yaw_noise);
  armor.ypr_in_world = {tools::limit_rad(a2c_yaw + d_yaw(rng)), kPitch, 0.0};
  return armor;
}

/// 生成第 frame 帧（frame 从 1 起）的真值：三板位姿 + 可见板 id
struct TrueFrame {
  std::array<Eigen::Vector3d, 3> pos;
  std::array<double, 3> a2c_yaw;
  int observed_id;
};

TrueFrame make_frame(double omega, int frame) {
  const double angle = omega * (frame * kDt);
  TrueFrame f;
  f.observed_id = 0;
  for (int i = 0; i < 3; ++i) {
    const auto [yaw_off, h_off] = outpost_const::kOffsetTable[i];
    const double yaw = tools::limit_rad(angle + yaw_off);
    f.a2c_yaw[i] = yaw;
    f.pos[i] = Eigen::Vector3d{2.0 - kRadius * std::cos(yaw),
                               -kRadius * std::sin(yaw), 2.0 + h_off};
  }
  // 可见板 = position.x 最小（距原点相机最近）
  for (int i = 1; i < 3; ++i) {
    if (f.pos[i].x() < f.pos[f.observed_id].x())
      f.observed_id = i;
  }
  return f;
}

struct SimResult {
  int switch_count = 0;
  int first_observed_id = 0;
  std::vector<int> observed_seq; // 每次切板后的可见板序列
};

/// 跑一整段仿真，返回统计信息
SimResult run_simulation(OutpostTarget &target, double omega, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> d_pos(0.0, kPosSigma);
  std::normal_distribution<double> d_yaw(0.0, kYawSigma);

  SimResult stat;
  int last_observed = -1;

  for (int frame = 1; frame <= kFrames; ++frame) {
    const TrueFrame f = make_frame(omega, frame);
    const int id = f.observed_id;

    if (frame == 1) {
      // 首帧已在测试主体用于构造 OutpostTarget，这里仅补记可见板
      last_observed = id;
      stat.first_observed_id = id;
      continue;
    }

    Armor armor =
        make_armor(f.pos[id], f.a2c_yaw[id], d_yaw(rng), d_pos(rng), rng);

    target.predict(kDt);
    target.update(armor);

    if (id != last_observed) {
      stat.switch_count++;
      stat.observed_seq.push_back(id);
      last_observed = id;
    }
  }
  return stat;
}

/// 用首帧真值板构造 OutpostTarget
std::chrono::steady_clock::time_point kT0 = std::chrono::steady_clock::now();

OutpostTarget make_target(double omega, unsigned seed) {
  std::mt19937 rng(seed);
  const TrueFrame f = make_frame(omega, 1);
  Armor armor = make_armor(f.pos[f.observed_id], f.a2c_yaw[f.observed_id], 0.0,
                           kPosSigma, rng);
  return OutpostTarget(armor, kT0);
}

} // namespace

int main() {
  // ===== 测试 1/2/3/4：正转 ω=+0.8π =====
  {
    const double omega = outpost_const::kAngularSpeed; // +0.8π
    OutpostTarget target = make_target(omega, 42);
    const SimResult stat = run_simulation(target, omega, 42);

    const double final_angle = omega * (kFrames * kDt);
    const Eigen::VectorXd x = target.ekf_x();

    // 首帧 angle≈0.025，最高板（id=0）正对相机 → 参考板绝对序号应为 0
    EXPECT_TRUE(stat.first_observed_id == 0);

    // --- 测试 1：EKF 收敛精度（docs 6.2 阈值）---
    EXPECT_TRUE(target.convergened());
    EXPECT_NEAR(x[0], 2.0, 0.05);
    EXPECT_NEAR(x[1], 0.0, 0.05);
    EXPECT_NEAR(x[2], 2.0, 0.05); // 参考板为最高板，z == 真值中心 z
    EXPECT_NEAR(x[3], omega, 0.1);
    // state[4] = 参考板 center→armor yaw = angle + π
    EXPECT_NEAR(tools::limit_rad(x[4] - (final_angle + kPi)), 0.0, 0.05);

    // --- 测试 2：切板方向锁定 ---
    // 注意：锁定符号取自相邻帧观测 yaw 跳变，与物理 ω 方向相反：
    // ω>0 时可见板序列 0→2→1（yaw 跳变 wrap 为 -120°），故锁定 -1。
    std::printf("[CCW] switch_count=%d\n", stat.switch_count);
    EXPECT_TRUE(stat.switch_count >= 6);
    EXPECT_TRUE(target.rotation_sign() == -1);
    // rel_see_index 始终折叠在 {-1,0,+1}
    EXPECT_TRUE(target.rel_see_index() >= -1 && target.rel_see_index() <= 1);
    // 正转可见板序列应为 2,1,0,2,1,0,...
    bool seq_ok = true;
    for (std::size_t k = 0; k < stat.observed_seq.size(); ++k) {
      if (stat.observed_seq[k] != ((2 - static_cast<int>(k) % 3) + 3) % 3)
        seq_ok = false;
    }
    EXPECT_TRUE(seq_ok);

    // --- 测试 3：三块板齐后 abs_ref_index == 0 ---
    EXPECT_TRUE(target.abs_ref_index() == 0);

    // --- 测试 4：armor_xyza_list 几何一致性 ---
    const std::vector<Eigen::Vector4d> list = target.armor_xyza_list();
    EXPECT_TRUE(list.size() == 3);
    for (const auto &v : list) {
      // 三板相对估计中心的水平距离恒为 radius
      const double rr = std::hypot(v[0] - x[0], v[1] - x[1]);
      EXPECT_NEAR(rr, kRadius, 0.005);
    }
    // 当前可见板位姿与真值一致
    const TrueFrame ff = make_frame(omega, kFrames);
    const Eigen::Vector3d true_pos = ff.pos[ff.observed_id];
    const Eigen::Vector4d &est = list[ff.observed_id];
    EXPECT_NEAR(est[0], true_pos.x(), 0.1);
    EXPECT_NEAR(est[1], true_pos.y(), 0.1);
    EXPECT_NEAR(est[2], true_pos.z(), 0.1);
    EXPECT_NEAR(tools::limit_rad(est[3] - ff.a2c_yaw[ff.observed_id]), 0.0,
                0.1);

    EXPECT_TRUE(!target.diverged());
  }

  // ===== 测试 2b：反转 ω=-0.8π，方向锁定 -1，序号仍为 0 =====
  {
    const double omega = -outpost_const::kAngularSpeed;
    OutpostTarget target = make_target(omega, 7);
    const SimResult stat = run_simulation(target, omega, 7);

    std::printf("[CW] switch_count=%d\n", stat.switch_count);
    EXPECT_TRUE(stat.switch_count >= 6);
    EXPECT_TRUE(target.rotation_sign() == +1);
    EXPECT_TRUE(target.abs_ref_index() == 0);
    EXPECT_TRUE(target.convergened());

    const Eigen::VectorXd x = target.ekf_x();
    EXPECT_NEAR(x[3], omega, 0.1);
    EXPECT_TRUE(!target.diverged());
  }

  // ===== 测试 5：观测位置门控丢弃大跳变帧 =====
  {
    const double omega = outpost_const::kAngularSpeed;
    OutpostTarget target = make_target(omega, 123);

    const Eigen::VectorXd x_before = target.ekf_x();

    // 构造一块 xy 偏移 0.5m 的无噪声观测（>0.10m 门限），yaw 不变
    TrueFrame f = make_frame(omega, 2);
    std::vector<cv::Point2f> kp(4);
    Armor bad(-1, 1.0f, cv::Rect(), kp);
    bad.name = ArmorName::outpost;
    bad.type = ArmorType::small;
    bad.xyz_in_world = f.pos[f.observed_id];
    bad.xyz_in_world.x() += 0.5;
    bad.ypr_in_world = {f.a2c_yaw[f.observed_id], kPitch, 0.0};
    target.predict(kDt);
    target.update(bad);

    const Eigen::VectorXd x_after = target.ekf_x();
    EXPECT_NEAR((x_after - x_before).norm(), 0.0, 1e-12);
  }

  std::printf("\n==== outpost_target_test: %d passed, %d failed ====\n", passed,
              failed);
  return failed == 0 ? 0 : 1;
}
