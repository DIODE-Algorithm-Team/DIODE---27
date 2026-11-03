// 弹道求解器测试（工作流 C / C5）
//
// 覆盖：
//   1. c=0 时与 tools::Trajectory 解析解一致（回归保护）
//   2. 阻力使 pitch/飞行时间变大（物理方向正确）
//   3. 与 1e-5 步长的高精度积分对比，端点误差 < 2 mm（对齐 txt benchmark）
//   4. 上坡/下坡目标
//   5. 不可解工况返回 false（不输出 NaN）
//   6. yaw 解析正确
//   7. 性能：100 次求解 < 50 ms

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "tools/ballistics.hpp"
#include "tools/trajectory.hpp"

namespace
{
int g_pass = 0;
int g_fail = 0;

void check(bool cond, const char * name, double got, double want, double tol)
{
  if (cond) {
    ++g_pass;
    std::printf("  [PASS] %s\n", name);
  } else {
    ++g_fail;
    std::printf("  [FAIL] %s  got=%.6f want=%.6f tol=%.6f\n", name, got, want, tol);
  }
}

void check(bool cond, const char * name)
{
  if (cond) {
    ++g_pass;
    std::printf("  [PASS] %s\n", name);
  } else {
    ++g_fail;
    std::printf("  [FAIL] %s\n", name);
  }
}

/// 独立的高精度积分（用于验证 solver 的端点是否真的命中）
struct RefResult
{
  bool reached = false;
  double z = 0.0;
  double t = 0.0;
};

RefResult reference_integrate(
  double pitch, double yaw, double v0, double d, double c, double g, double h = 1e-5)
{
  RefResult out;
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel{v0 * cp * cy, v0 * cp * sy, v0 * sp};
  double t = 0.0;
  const double max_t = 2.0;

  while (t < max_t) {
    const double xy0 = std::hypot(pos.x(), pos.y());
    if (xy0 >= d) {
      out.reached = true;
      out.z = pos.z();
      out.t = t;
      return out;
    }
    const Eigen::Vector3d p0 = pos;
    const double t0 = t;

    const auto acc = [&](const Eigen::Vector3d & v) {
      const double s = v.norm();
      return Eigen::Vector3d(-c * s * v.x(), -c * s * v.y(), -g - c * s * v.z());
    };

    const Eigen::Vector3d k1p = vel, k1v = acc(vel);
    const Eigen::Vector3d v2 = vel + 0.5 * h * k1v;
    const Eigen::Vector3d k2p = v2, k2v = acc(v2);
    const Eigen::Vector3d v3 = vel + 0.5 * h * k2v;
    const Eigen::Vector3d k3p = v3, k3v = acc(v3);
    const Eigen::Vector3d v4 = vel + h * k3v;
    const Eigen::Vector3d k4p = v4, k4v = acc(v4);

    pos += h / 6.0 * (k1p + 2.0 * k2p + 2.0 * k3p + k4p);
    vel += h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
    t += h;

    const double xy1 = std::hypot(pos.x(), pos.y());
    if (xy1 >= d) {
      const double denom = xy1 - xy0;
      const double ratio = denom > 1e-12 ? (d - xy0) / denom : 1.0;
      out.reached = true;
      out.z = p0.z() + ratio * (pos.z() - p0.z());
      out.t = t0 + ratio * (t - t0);
      return out;
    }
  }
  return out;
}

}  // namespace

int main()
{
  std::printf("=== ballistic_solver_test ===\n");

  // -------------------------------------------------------------------------
  std::printf("\n[1] c=0 与解析解一致\n");
  for (double v0 : {15.0, 20.0, 25.0, 30.0}) {
    for (double d : {3.0, 5.0, 8.0, 10.0, 15.0}) {
      for (double z : {-0.5, 0.0, 0.8}) {
        tools::BallisticParams p;
        p.resistance = 0.0;

        const auto sol = tools::solve_ballistic(v0, {d, 0.0, z}, p);
        tools::Trajectory analytic(v0, d, z);

        const double diff = std::abs(sol.pitch - analytic.pitch);
        check(sol.solvable, "真空可解", sol.solvable ? 1 : 0, 1, 0);
        check(diff < 2e-4, "pitch 与解析解一致", diff, 0.0, 2e-4);
      }
    }
  }

  // -------------------------------------------------------------------------
  std::printf("\n[2] 阻力使 pitch / 飞行时间变大\n");
  {
    const double v0 = 25.0, d = 10.0;
    tools::BallisticParams vacuum;
    tools::BallisticParams drag;
    drag.resistance = 0.0201;

    const auto a = tools::solve_ballistic(v0, {d, 0.0, 0.0}, vacuum);
    const auto b = tools::solve_ballistic(v0, {d, 0.0, 0.0}, drag);

    check(a.solvable && b.solvable, "两者均可解");
    check(b.pitch > a.pitch + 0.005, "阻力下 pitch 更大", b.pitch, a.pitch, 0.005);
    check(b.fly_time > a.fly_time, "阻力下飞行时间更长", b.fly_time, a.fly_time, 0.0);
  }

  // -------------------------------------------------------------------------
  std::printf("\n[3] 与高精度积分对比（端点误差）\n");
  {
    double worst = 0.0;
    int skipped = 0;
    for (double v0 : {15.0, 20.0, 25.0, 30.0}) {
      for (double d : {3.0, 5.0, 8.0, 10.0, 15.0, 20.0}) {
        // 真空最大射程 v0²/g；接近或超过该距离属于物理不可达，跳过（[5] 单独测不可解）
        const double max_range = v0 * v0 / 9.7833;
        if (d > 0.95 * max_range) {
          ++skipped;
          continue;
        }
        for (double c : {0.003, 0.010, 0.0201}) {
          for (double z : {-0.8, 0.0, 1.2}) {
            tools::BallisticParams p;
            p.resistance = c;
            const auto sol = tools::solve_ballistic(v0, {d, 0.0, z}, p);
            if (!sol.solvable) {
              // 接近射程极限（阻力把可达距离压到 d 以下）属物理不可达，跳过；其余算失败
              if (d > 0.7 * max_range) {
                ++skipped;
                continue;
              }
              ++g_fail;
              std::printf("  [FAIL] 不可解 v0=%.0f d=%.1f c=%.3f z=%.1f\n", v0, d, c, z);
              continue;
            }
            const auto ref = reference_integrate(
              sol.pitch, sol.yaw, v0, d, p.resistance, p.gravity);
            const double err = std::abs(ref.z - z);
            worst = std::max(worst, err);
          }
        }
      }
    }
    check(worst < 2e-3, "最大端点误差 < 2 mm", worst * 1000.0, 0.0, 2.0);
    std::printf("       实测最大误差 %.3f mm（跳过 %d 组接近射程极限）\n", worst * 1000.0, skipped);
  }

  // -------------------------------------------------------------------------
  std::printf("\n[4] 上坡 / 下坡\n");
  {
    tools::BallisticParams p;
    p.resistance = 0.01;

    const auto up = tools::solve_ballistic(25.0, {5.0, 0.0, 1.5}, p);
    const auto flat = tools::solve_ballistic(25.0, {5.0, 0.0, 0.0}, p);
    const auto down = tools::solve_ballistic(25.0, {5.0, 0.0, -1.0}, p);

    check(up.solvable && down.solvable, "上下坡可解");
    check(up.pitch > flat.pitch, "上坡 pitch 更大", up.pitch, flat.pitch, 0.0);
    check(down.pitch < flat.pitch, "下坡 pitch 更小", down.pitch, flat.pitch, 0.0);
  }

  // -------------------------------------------------------------------------
  std::printf("\n[5] 不可解工况\n");
  {
    tools::BallisticParams p;
    p.resistance = 0.01;
    check(!tools::solve_ballistic(0.0, {5.0, 0.0, 0.0}, p).solvable, "v0=0 不可解");
    check(!tools::solve_ballistic(5.0, {400.0, 0.0, 0.0}, p).solvable, "超远射程不可解");
    check(!tools::solve_ballistic(8.0, {60.0, 0.0, 0.0}, p).solvable, "低初速远距不可解");

    const auto sol = tools::solve_ballistic(0.0, {5.0, 0.0, 0.0}, p);
    check(std::isfinite(sol.pitch) && std::isfinite(sol.fly_time), "失败时无 NaN");
  }

  // -------------------------------------------------------------------------
  std::printf("\n[6] yaw 与单调性\n");
  {
    tools::BallisticParams p;
    p.resistance = 0.012;

    const auto s = tools::solve_ballistic(25.0, {3.0, 4.0, 0.0}, p);
    check(std::abs(s.yaw - std::atan2(4.0, 3.0)) < 1e-9, "yaw = atan2(y,x)", s.yaw, std::atan2(4.0, 3.0), 1e-9);

    double prev = -1.0;
    bool monotonic = true;
    for (double d : {3.0, 5.0, 8.0, 10.0, 15.0, 20.0}) {
      const auto r = tools::solve_ballistic(30.0, {d, 0.0, 0.0}, p);
      if (!r.solvable) continue;
      if (r.pitch < prev) monotonic = false;
      prev = r.pitch;
    }
    check(monotonic, "pitch 随距离单调不减");
  }

  // -------------------------------------------------------------------------
  std::printf("\n[7] 性能\n");
  {
    tools::BallisticParams p;
    p.resistance = 0.012;

    const auto t0 = std::chrono::steady_clock::now();
    int n = 0;
    for (int i = 0; i < 100; ++i) {
      const auto s = tools::solve_ballistic(25.0, {4.0 + 0.1 * i, 0.5, 0.2}, p);
      n += s.solvable ? 1 : 0;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("       100 次求解 %.2f ms（可解 %d）\n", ms, n);
    check(ms < 50.0, "100 次求解 < 50 ms", ms, 0.0, 50.0);
  }

  std::printf("\n=== PASS: %d, FAIL: %d ===\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
