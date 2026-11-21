#include "roboticscore/planning/kinodynamic/reeds_shepp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/models/unicycle.hpp"

namespace roboticscore::planning {
namespace {

using RS = RsSegment;
constexpr double kPi = std::numbers::pi;
constexpr double kHalfPi = 0.5 * std::numbers::pi;
constexpr double kZero = 1e-10;

// Wraps to (-pi, pi], the convention of the closed-form solutions below.
double mod2pi(double a) {
  double v = std::fmod(a, 2.0 * kPi);
  if (v < -kPi) {
    v += 2.0 * kPi;
  } else if (v > kPi) {
    v -= 2.0 * kPi;
  }
  return v;
}

void polar(double x, double y, double& r, double& theta) {
  r = std::hypot(x, y);
  theta = std::atan2(y, x);
}

// ---------------------------------------------------------------- base-word solutions
// Formula numbers refer to Reeds & Shepp (1990), section 8. Lengths are for unit radius.

bool lp_sp_lp(double x, double y, double phi, double& t, double& u, double& v) {  // 8.1
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  if (t >= -kZero) {
    v = mod2pi(phi - t);
    return v >= -kZero;
  }
  return false;
}

bool lp_sp_rp(double x, double y, double phi, double& t, double& u, double& v) {  // 8.2
  double t1 = 0.0;
  double u1 = 0.0;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
  u1 *= u1;
  if (u1 >= 4.0) {
    u = std::sqrt(u1 - 4.0);
    const double theta = std::atan2(2.0, u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    return t >= -kZero && v >= -kZero;
  }
  return false;
}

bool lp_rm_l(double x, double y, double phi, double& t, double& u, double& v) {  // 8.3 / 8.4
  const double xi = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  double u1 = 0.0;
  double theta = 0.0;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.0) {
    u = -2.0 * std::asin(0.25 * u1);
    t = mod2pi(theta + 0.5 * u + kPi);
    v = mod2pi(phi - t + u);
    return t >= -kZero && u <= kZero;
  }
  return false;
}

void tau_omega(double u, double v, double xi, double eta, double phi, double& tau, double& omega) {
  const double delta = mod2pi(u - v);
  const double a = std::sin(u) - std::sin(delta);
  const double b = std::cos(u) - std::cos(delta) - 1.0;
  const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
  const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = (t2 < 0.0) ? mod2pi(t1 + kPi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

bool lp_rup_lum_rm(double x, double y, double phi, double& t, double& u, double& v) {  // 8.7
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  if (rho <= 1.0) {
    u = std::acos(rho);
    tau_omega(u, -u, xi, eta, phi, t, v);
    return t >= -kZero && v <= kZero;
  }
  return false;
}

bool lp_rum_lum_rp(double x, double y, double phi, double& t, double& u, double& v) {  // 8.8
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho >= 0.0 && rho <= 1.0) {
    u = -std::acos(rho);
    if (u >= -kHalfPi) {
      tau_omega(u, u, xi, eta, phi, t, v);
      return t >= -kZero && v >= -kZero;
    }
  }
  return false;
}

bool lp_rm_sm_lm(double x, double y, double phi, double& t, double& u, double& v) {  // 8.9
  const double xi = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0) {
    const double r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = mod2pi(theta + std::atan2(r, -2.0));
    v = mod2pi(phi - kHalfPi - t);
    return t >= -kZero && u <= kZero && v <= kZero;
  }
  return false;
}

bool lp_rm_sm_rm(double x, double y, double phi, double& t, double& u, double& v) {  // 8.10
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.0) {
    t = theta;
    u = 2.0 - rho;
    v = mod2pi(t + kHalfPi - phi);
    return t >= -kZero && u <= kZero && v <= kZero;
  }
  return false;
}

bool lp_rm_s_lm_rp(double x, double y, double phi, double& t, double& u, double& v) {  // 8.11
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= kZero) {
      t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = mod2pi(t - phi);
      return t >= -kZero && v >= -kZero;
    }
  }
  return false;
}

// ---------------------------------------------------------------- candidate collection

struct Collector {
  double x;
  double y;
  double phi;
  std::vector<ReedsSheppPath> out;

  // Integrates a unit-radius word from the origin and keeps it only if it reaches the target.
  void add(std::array<RS, 5> types, std::array<double, 5> lengths, int n) {
    double px = 0.0;
    double py = 0.0;
    double th = 0.0;
    for (int i = 0; i < n; ++i) {
      const double l = lengths[static_cast<std::size_t>(i)];
      switch (types[static_cast<std::size_t>(i)]) {
        case RS::kLeft:
          px += std::sin(th + l) - std::sin(th);
          py += -std::cos(th + l) + std::cos(th);
          th += l;
          break;
        case RS::kRight:
          px += -std::sin(th - l) + std::sin(th);
          py += std::cos(th - l) - std::cos(th);
          th -= l;
          break;
        case RS::kStraight:
          px += l * std::cos(th);
          py += l * std::sin(th);
          break;
      }
    }
    const double scale = std::max(1.0, std::hypot(x, y));
    if (std::hypot(px - x, py - y) > 1e-6 * scale
        || std::abs(math::angle_difference(th, phi)) > 1e-6) {
      return;  // numerically invalid candidate (should not happen for valid formulas)
    }
    for (int i = n; i < 5; ++i) {
      lengths[static_cast<std::size_t>(i)] = 0.0;
      types[static_cast<std::size_t>(i)] = RS::kStraight;
    }
    ReedsSheppPath p;
    p.types = types;
    p.lengths = lengths;
    out.push_back(p);
  }

  void csc() {
    double t = 0, u = 0, v = 0;
    const std::array<RS, 5> lsl{RS::kLeft, RS::kStraight, RS::kLeft};
    const std::array<RS, 5> rsr{RS::kRight, RS::kStraight, RS::kRight};
    const std::array<RS, 5> lsr{RS::kLeft, RS::kStraight, RS::kRight};
    const std::array<RS, 5> rsl{RS::kRight, RS::kStraight, RS::kLeft};
    if (lp_sp_lp(x, y, phi, t, u, v)) add(lsl, {t, u, v}, 3);
    if (lp_sp_lp(-x, y, -phi, t, u, v)) add(lsl, {-t, -u, -v}, 3);  // timeflip
    if (lp_sp_lp(x, -y, -phi, t, u, v)) add(rsr, {t, u, v}, 3);     // reflect
    if (lp_sp_lp(-x, -y, phi, t, u, v)) add(rsr, {-t, -u, -v}, 3);  // timeflip + reflect
    if (lp_sp_rp(x, y, phi, t, u, v)) add(lsr, {t, u, v}, 3);
    if (lp_sp_rp(-x, y, -phi, t, u, v)) add(lsr, {-t, -u, -v}, 3);
    if (lp_sp_rp(x, -y, -phi, t, u, v)) add(rsl, {t, u, v}, 3);
    if (lp_sp_rp(-x, -y, phi, t, u, v)) add(rsl, {-t, -u, -v}, 3);
  }

  void ccc() {
    double t = 0, u = 0, v = 0;
    const std::array<RS, 5> lrl{RS::kLeft, RS::kRight, RS::kLeft};
    const std::array<RS, 5> rlr{RS::kRight, RS::kLeft, RS::kRight};
    if (lp_rm_l(x, y, phi, t, u, v)) add(lrl, {t, u, v}, 3);
    if (lp_rm_l(-x, y, -phi, t, u, v)) add(lrl, {-t, -u, -v}, 3);
    if (lp_rm_l(x, -y, -phi, t, u, v)) add(rlr, {t, u, v}, 3);
    if (lp_rm_l(-x, -y, phi, t, u, v)) add(rlr, {-t, -u, -v}, 3);
    // Backwards: the same word traversed from the goal.
    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);
    if (lp_rm_l(xb, yb, phi, t, u, v)) add(lrl, {v, u, t}, 3);
    if (lp_rm_l(-xb, yb, -phi, t, u, v)) add(lrl, {-v, -u, -t}, 3);
    if (lp_rm_l(xb, -yb, -phi, t, u, v)) add(rlr, {v, u, t}, 3);
    if (lp_rm_l(-xb, -yb, phi, t, u, v)) add(rlr, {-v, -u, -t}, 3);
  }

  void cccc() {
    double t = 0, u = 0, v = 0;
    const std::array<RS, 5> lrlr{RS::kLeft, RS::kRight, RS::kLeft, RS::kRight};
    const std::array<RS, 5> rlrl{RS::kRight, RS::kLeft, RS::kRight, RS::kLeft};
    if (lp_rup_lum_rm(x, y, phi, t, u, v)) add(lrlr, {t, u, -u, v}, 4);
    if (lp_rup_lum_rm(-x, y, -phi, t, u, v)) add(lrlr, {-t, -u, u, -v}, 4);
    if (lp_rup_lum_rm(x, -y, -phi, t, u, v)) add(rlrl, {t, u, -u, v}, 4);
    if (lp_rup_lum_rm(-x, -y, phi, t, u, v)) add(rlrl, {-t, -u, u, -v}, 4);
    if (lp_rum_lum_rp(x, y, phi, t, u, v)) add(lrlr, {t, u, u, v}, 4);
    if (lp_rum_lum_rp(-x, y, -phi, t, u, v)) add(lrlr, {-t, -u, -u, -v}, 4);
    if (lp_rum_lum_rp(x, -y, -phi, t, u, v)) add(rlrl, {t, u, u, v}, 4);
    if (lp_rum_lum_rp(-x, -y, phi, t, u, v)) add(rlrl, {-t, -u, -u, -v}, 4);
  }

  void ccsc() {
    double t = 0, u = 0, v = 0;
    const std::array<RS, 5> lrsl{RS::kLeft, RS::kRight, RS::kStraight, RS::kLeft};
    const std::array<RS, 5> rlsr{RS::kRight, RS::kLeft, RS::kStraight, RS::kRight};
    const std::array<RS, 5> lrsr{RS::kLeft, RS::kRight, RS::kStraight, RS::kRight};
    const std::array<RS, 5> rlsl{RS::kRight, RS::kLeft, RS::kStraight, RS::kLeft};
    if (lp_rm_sm_lm(x, y, phi, t, u, v)) add(lrsl, {t, -kHalfPi, u, v}, 4);
    if (lp_rm_sm_lm(-x, y, -phi, t, u, v)) add(lrsl, {-t, kHalfPi, -u, -v}, 4);
    if (lp_rm_sm_lm(x, -y, -phi, t, u, v)) add(rlsr, {t, -kHalfPi, u, v}, 4);
    if (lp_rm_sm_lm(-x, -y, phi, t, u, v)) add(rlsr, {-t, kHalfPi, -u, -v}, 4);
    if (lp_rm_sm_rm(x, y, phi, t, u, v)) add(lrsr, {t, -kHalfPi, u, v}, 4);
    if (lp_rm_sm_rm(-x, y, -phi, t, u, v)) add(lrsr, {-t, kHalfPi, -u, -v}, 4);
    if (lp_rm_sm_rm(x, -y, -phi, t, u, v)) add(rlsl, {t, -kHalfPi, u, v}, 4);
    if (lp_rm_sm_rm(-x, -y, phi, t, u, v)) add(rlsl, {-t, kHalfPi, -u, -v}, 4);
    // Backwards variants (CSCC words).
    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);
    const std::array<RS, 5> lsrl{RS::kLeft, RS::kStraight, RS::kRight, RS::kLeft};
    const std::array<RS, 5> rslr{RS::kRight, RS::kStraight, RS::kLeft, RS::kRight};
    const std::array<RS, 5> rsrl{RS::kRight, RS::kStraight, RS::kRight, RS::kLeft};
    const std::array<RS, 5> lslr{RS::kLeft, RS::kStraight, RS::kLeft, RS::kRight};
    if (lp_rm_sm_lm(xb, yb, phi, t, u, v)) add(lsrl, {v, u, -kHalfPi, t}, 4);
    if (lp_rm_sm_lm(-xb, yb, -phi, t, u, v)) add(lsrl, {-v, -u, kHalfPi, -t}, 4);
    if (lp_rm_sm_lm(xb, -yb, -phi, t, u, v)) add(rslr, {v, u, -kHalfPi, t}, 4);
    if (lp_rm_sm_lm(-xb, -yb, phi, t, u, v)) add(rslr, {-v, -u, kHalfPi, -t}, 4);
    if (lp_rm_sm_rm(xb, yb, phi, t, u, v)) add(rsrl, {v, u, -kHalfPi, t}, 4);
    if (lp_rm_sm_rm(-xb, yb, -phi, t, u, v)) add(rsrl, {-v, -u, kHalfPi, -t}, 4);
    if (lp_rm_sm_rm(xb, -yb, -phi, t, u, v)) add(lslr, {v, u, -kHalfPi, t}, 4);
    if (lp_rm_sm_rm(-xb, -yb, phi, t, u, v)) add(lslr, {-v, -u, kHalfPi, -t}, 4);
  }

  void ccscc() {
    double t = 0, u = 0, v = 0;
    const std::array<RS, 5> lrslr{RS::kLeft, RS::kRight, RS::kStraight, RS::kLeft, RS::kRight};
    const std::array<RS, 5> rlsrl{RS::kRight, RS::kLeft, RS::kStraight, RS::kRight, RS::kLeft};
    if (lp_rm_s_lm_rp(x, y, phi, t, u, v)) add(lrslr, {t, -kHalfPi, u, -kHalfPi, v}, 5);
    if (lp_rm_s_lm_rp(-x, y, -phi, t, u, v)) add(lrslr, {-t, kHalfPi, -u, kHalfPi, -v}, 5);
    if (lp_rm_s_lm_rp(x, -y, -phi, t, u, v)) add(rlsrl, {t, -kHalfPi, u, -kHalfPi, v}, 5);
    if (lp_rm_s_lm_rp(-x, -y, phi, t, u, v)) add(rlsrl, {-t, kHalfPi, -u, kHalfPi, -v}, 5);
  }
};

double curvature_sign(RS type) {
  switch (type) {
    case RS::kLeft:
      return 1.0;
    case RS::kRight:
      return -1.0;
    case RS::kStraight:
      return 0.0;
  }
  return 0.0;
}

}  // namespace

double ReedsSheppPath::length() const {
  double sum = 0.0;
  for (const double l : lengths) {
    sum += std::abs(l);
  }
  return sum * radius;
}

int ReedsSheppPath::reversals() const {
  int count = 0;
  double prev = 0.0;
  for (const double l : lengths) {
    if (std::abs(l) < kZero) continue;
    if (prev != 0.0 && (l > 0.0) != (prev > 0.0)) ++count;
    prev = l;
  }
  return count;
}

std::vector<ReedsSheppPath> reeds_shepp_candidates(const geometry::Pose2& start,
                                                   const geometry::Pose2& goal, double radius) {
  detail::require(radius > 0.0, "reeds_shepp: radius must be positive");
  const geometry::Pose2 rel = start.between(goal);
  Collector c{rel.x() / radius, rel.y() / radius, rel.theta(), {}};
  c.csc();
  c.ccc();
  c.cccc();
  c.ccsc();
  c.ccscc();
  for (auto& p : c.out) {
    p.radius = radius;
  }
  return c.out;
}

ReedsSheppPath reeds_shepp_shortest(const geometry::Pose2& start, const geometry::Pose2& goal,
                                    double radius) {
  const auto candidates = reeds_shepp_candidates(start, goal, radius);
  if (candidates.empty()) {
    // Unreachable in exact arithmetic (Reeds & Shepp prove completeness); keep a safe value.
    throw NumericalError("reeds_shepp_shortest: no valid word found");
  }
  return *std::min_element(
      candidates.begin(), candidates.end(),
      [](const ReedsSheppPath& a, const ReedsSheppPath& b) { return a.length() < b.length(); });
}

std::vector<DirectedPose> sample_reeds_shepp(const geometry::Pose2& start,
                                             const ReedsSheppPath& path, double step) {
  detail::require(step > 0.0, "sample_reeds_shepp: step must be positive");
  std::vector<DirectedPose> out;
  geometry::Pose2 pose = start;
  bool first = true;
  for (std::size_t i = 0; i < path.types.size(); ++i) {
    const double len = path.lengths[i] * path.radius;  // signed metres
    if (std::abs(len) < 1e-12) continue;
    const double kappa = curvature_sign(path.types[i]) / path.radius;
    const bool forward = len > 0.0;
    if (first) {
      out.push_back({pose, forward, kappa});
      first = false;
    }
    const double dist = std::abs(len);
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
    const geometry::Pose2 seg_start = pose;
    for (int k = 1; k <= n; ++k) {
      const double s = (forward ? 1.0 : -1.0) * dist * k / n;
      pose = models::Unicycle::step(seg_start, {s, s * kappa}, 1.0);
      out.push_back({pose, forward, kappa});
    }
  }
  if (out.empty()) {
    out.push_back({start, true, 0.0});
  }
  return out;
}

}  // namespace roboticscore::planning
