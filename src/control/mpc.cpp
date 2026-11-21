#include "roboticscore/control/mpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"

namespace roboticscore::control {
namespace {

constexpr Index kNx = 4;
constexpr Index kNu = 2;

}  // namespace

MpcTracker::MpcTracker(const models::BicycleParams& vehicle, const MpcConfig& config)
    : PathTracker(vehicle, PidConfig{}), config_(config), model_(vehicle) {
  detail::require(config.horizon >= 2, "MpcTracker: horizon must be at least 2");
  detail::require(config.dt > 0.0, "MpcTracker: dt must be positive");
  detail::require((config.state_weights.array() >= 0.0).all()
                      && (config.input_weights.array() > 0.0).all()
                      && (config.input_rate_weights.array() >= 0.0).all(),
                  "MpcTracker: weights must be non-negative (input weights positive)");
  detail::require(config.max_steer_rate > 0.0, "MpcTracker: max_steer_rate must be positive");
  detail::require(config.min_speed < vehicle.max_speed, "MpcTracker: invalid speed bounds");
}

void MpcTracker::reset() {
  PathTracker::reset();
  previous_u_.resize(0);
  previous_y_.resize(0);
  last_input_.setZero();
}

VehicleCommand MpcTracker::compute(const models::BicycleState& state,
                                   const trajectory::ReferencePath& path, double dt) {
  detail::require(dt > 0.0, "MpcTracker::compute: dt must be positive");
  const int n = config_.horizon;
  const double h = config_.dt;
  const auto proj = track_point(Vec2(state(0), state(1)), state(2), path);
  status_.target = proj.point;

  // 1. Reference states r_0..r_N and feedforward inputs along the path.
  std::vector<Vec4> ref(static_cast<std::size_t>(n) + 1);
  std::vector<Vec2> u_ref(static_cast<std::size_t>(n));
  double s = proj.s;
  double yaw_prev = state(2);
  for (int k = 0; k <= n; ++k) {
    const trajectory::PathPoint p = path.at(s);
    const double yaw = yaw_prev + math::angle_difference(p.heading, yaw_prev);  // unwrap
    yaw_prev = yaw;
    ref[static_cast<std::size_t>(k)] = Vec4(p.position.x(), p.position.y(), yaw, p.speed);
    if (k < n) {
      u_ref[static_cast<std::size_t>(k)] = Vec2(0.0, std::atan(vehicle_.wheelbase * p.curvature));
    }
    // Advance by the distance travelled at the reference speed (at least a little so the
    // horizon keeps looking ahead when the reference starts at rest).
    s += std::max(std::abs(p.speed), 0.5) * h;
  }
  for (int k = 0; k < n; ++k) {
    const auto ku = static_cast<std::size_t>(k);
    u_ref[ku](0) = (ref[ku + 1](3) - ref[ku](3)) / h;
  }

  // 2-3. Condensed prediction X = Phi x0 + Gamma U + Psi, built recursively.
  const Index nx_all = kNx * n;
  const Index nu_all = kNu * n;
  MatX gamma = MatX::Zero(nx_all, nu_all);
  VecX free_resp(nx_all);  // Phi x0 + Psi
  Vec4 x_free = state;
  MatX gamma_row = MatX::Zero(kNx, nu_all);  // d x_k / d U
  for (int k = 0; k < n; ++k) {
    const auto ku = static_cast<std::size_t>(k);
    const models::BicycleLinearization lin = model_.linearize(ref[ku], u_ref[ku], h);
    x_free = lin.a * x_free + lin.c;
    gamma_row = lin.a * gamma_row;
    gamma_row.block(0, kNu * k, kNx, kNu) = lin.b;
    gamma.middleRows(kNx * k, kNx) = gamma_row;
    free_resp.segment(kNx * k, kNx) = x_free;
  }

  // Stacked weights and references.
  VecX q_diag(nx_all);
  VecX x_ref(nx_all);
  VecX r_diag(nu_all);
  VecX rd_diag(nu_all);
  VecX u_ref_all(nu_all);
  for (int k = 0; k < n; ++k) {
    const double scale = (k == n - 1) ? config_.terminal_weight_scale : 1.0;
    q_diag.segment(kNx * k, kNx) = scale * config_.state_weights;
    x_ref.segment(kNx * k, kNx) = ref[static_cast<std::size_t>(k) + 1];
    r_diag.segment(kNu * k, kNu) = config_.input_weights;
    rd_diag.segment(kNu * k, kNu) = config_.input_rate_weights;
    u_ref_all.segment(kNu * k, kNu) = u_ref[static_cast<std::size_t>(k)];
  }
  // Difference operator D U - d0 = (u_0 - u_prev, u_1 - u_0, ...).
  MatX d = MatX::Identity(nu_all, nu_all);
  for (Index i = kNu; i < nu_all; ++i) {
    d(i, i - kNu) = -1.0;
  }
  VecX d0 = VecX::Zero(nu_all);
  d0.head<kNu>() = last_input_;

  optimization::QpProblem qp;
  qp.p = 2.0
         * (gamma.transpose() * q_diag.asDiagonal() * gamma + MatX(r_diag.asDiagonal())
            + d.transpose() * rd_diag.asDiagonal() * d);
  qp.q = 2.0
         * (gamma.transpose() * q_diag.asDiagonal() * (free_resp - x_ref)
            - r_diag.asDiagonal() * u_ref_all - d.transpose() * rd_diag.asDiagonal() * d0);

  // Constraints: input bounds, steering-rate bounds, speed bounds.
  const Index m = nu_all + n + n;
  qp.a = MatX::Zero(m, nu_all);
  qp.l = VecX(m);
  qp.u = VecX(m);
  qp.a.topRows(nu_all) = MatX::Identity(nu_all, nu_all);
  for (int k = 0; k < n; ++k) {
    qp.l(kNu * k) = -vehicle_.max_accel;
    qp.u(kNu * k) = vehicle_.max_accel;
    qp.l(kNu * k + 1) = -vehicle_.max_steer;
    qp.u(kNu * k + 1) = vehicle_.max_steer;
    const Index row = nu_all + k;
    qp.a.row(row) = d.row(kNu * k + 1);
    // The first move happens after one control period dt, later ones after the prediction
    // step h; using h for the first row would let the applied steering change too fast when
    // the controller runs faster than its model.
    const double rate = config_.max_steer_rate * (k == 0 ? dt : h);
    qp.l(row) = -rate + d0(kNu * k + 1);
    qp.u(row) = rate + d0(kNu * k + 1);
    const Index vrow = nu_all + n + k;
    qp.a.row(vrow) = gamma.row(kNx * k + 3);
    qp.l(vrow) = config_.min_speed - free_resp(kNx * k + 3);
    qp.u(vrow) = vehicle_.max_speed - free_resp(kNx * k + 3);
  }
  // The steering-rate bound for step 0 is relative to the applied steering; if the previous
  // input lies outside the box (e.g. after a reset) keep the problem feasible.
  for (Index i = 0; i < m; ++i) {
    if (qp.l(i) > qp.u(i)) std::swap(qp.l(i), qp.u(i));
  }

  // Warm start: shift the previous solution by one step.
  VecX warm;
  const VecX* warm_ptr = nullptr;
  const VecX* warm_y_ptr = nullptr;
  if (previous_u_.size() == nu_all) {
    warm = previous_u_;
    warm.head(nu_all - kNu) = previous_u_.tail(nu_all - kNu);
    warm_ptr = &warm;
    if (previous_y_.size() == m) warm_y_ptr = &previous_y_;
  }
  const optimization::QpResult sol = optimization::solve_qp(qp, config_.qp, warm_ptr, warm_y_ptr);
  qp_iterations_ = sol.iterations;
  qp_solved_ = sol.solved();
  previous_u_ = sol.x;
  previous_y_ = sol.y;

  // Predicted positions for visualisation.
  const VecX x_pred = free_resp + gamma * sol.x;
  status_.prediction.clear();
  for (int k = 0; k < n; ++k) {
    status_.prediction.emplace_back(x_pred(kNx * k), x_pred(kNx * k + 1));
  }

  VehicleCommand cmd;
  cmd.acceleration = std::clamp(sol.x(0), -vehicle_.max_accel, vehicle_.max_accel);
  cmd.steering = clamp_steering(sol.x(1));
  last_input_ = Vec2(cmd.acceleration, cmd.steering);
  return cmd;
}

}  // namespace roboticscore::control
