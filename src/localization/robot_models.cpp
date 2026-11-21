#include "roboticscore/localization/robot_models.hpp"

#include <cmath>
#include <numbers>
#include <vector>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/models/unicycle.hpp"

namespace roboticscore::localization {
namespace {

// Weighted mean where component `angle_index` is averaged on the circle.
VecX mean_with_angle(const MatX& points, const VecX& weights, Index angle_index) {
  detail::require(points.cols() == weights.size(), "mean: one weight per point is required");
  VecX m = points * weights;
  double s = 0.0;
  double c = 0.0;
  for (Index i = 0; i < points.cols(); ++i) {
    s += weights(i) * std::sin(points(angle_index, i));
    c += weights(i) * std::cos(points(angle_index, i));
  }
  m(angle_index) = math::wrap_to_pi(std::atan2(s, c));
  return m;
}

}  // namespace

Mat2 VelocityNoise::control_covariance(double v, double omega) const {
  const double v2 = v * v;
  const double w2 = omega * omega;
  Mat2 m = Mat2::Zero();
  m(0, 0) = alpha1 * v2 + alpha2 * w2 + min_var_v;
  m(1, 1) = alpha3 * v2 + alpha4 * w2 + min_var_omega;
  return m;
}

VecX pose_residual(const VecX& a, const VecX& b) {
  VecX d = a - b;
  d(2) = math::wrap_to_pi(d(2));
  return d;
}

VecX normalize_pose(const VecX& x) {
  VecX out = x;
  out(2) = math::wrap_to_pi(out(2));
  return out;
}

VecX pose_mean(const MatX& points, const VecX& weights) {
  return mean_with_angle(points, weights, 2);
}

estimation::ProcessModel unicycle_process_model(const VelocityNoise& noise) {
  estimation::ProcessModel model;
  model.f = [](const VecX& x, const VecX& u, double dt) -> VecX {
    return models::Unicycle::step(Vec3(x), {u(0), u(1)}, dt);
  };
  model.jacobian = [](const VecX& x, const VecX& u, double dt) -> MatX {
    return models::Unicycle::jacobians(Vec3(x), {u(0), u(1)}, dt).wrt_state;
  };
  model.noise = [noise](const VecX& x, const VecX& u, double dt) -> MatX {
    const auto j = models::Unicycle::jacobians(Vec3(x), {u(0), u(1)}, dt);
    // Tiny additive floor keeps P full rank even for perfectly straight motion.
    return j.wrt_input * noise.control_covariance(u(0), u(1)) * j.wrt_input.transpose()
           + 1e-12 * Mat3::Identity();
  };
  model.residual = pose_residual;
  model.normalize = normalize_pose;
  model.mean = pose_mean;
  return model;
}

Vec2 predict_range_bearing(const Vec3& pose, const Vec2& landmark) {
  const double dx = landmark.x() - pose.x();
  const double dy = landmark.y() - pose.y();
  return {std::hypot(dx, dy), math::wrap_to_pi(std::atan2(dy, dx) - pose.z())};
}

estimation::MeasurementModel range_bearing_model(const Vec2& landmark, double range_std,
                                                 double bearing_std) {
  detail::require(range_std > 0.0 && bearing_std > 0.0,
                  "range_bearing_model: noise standard deviations must be positive");
  estimation::MeasurementModel model;
  model.h = [landmark](const VecX& x) -> VecX {
    return predict_range_bearing(Vec3(x), landmark);
  };
  model.jacobian = [landmark](const VecX& x) -> MatX {
    const double dx = landmark.x() - x(0);
    const double dy = landmark.y() - x(1);
    const double q = dx * dx + dy * dy;
    const double r = std::sqrt(q);
    if (r < 1e-9) {
      throw NumericalError("range_bearing_model: Jacobian undefined at zero range");
    }
    MatX h(2, 3);
    h << -dx / r, -dy / r, 0.0,  //
        dy / q, -dx / q, -1.0;
    return h;
  };
  model.noise = Vec2(range_std * range_std, bearing_std * bearing_std).asDiagonal();
  model.residual = [](const VecX& a, const VecX& b) -> VecX {
    VecX d = a - b;
    d(1) = math::wrap_to_pi(d(1));
    return d;
  };
  model.mean = [](const MatX& points, const VecX& weights) -> VecX {
    return mean_with_angle(points, weights, 1);
  };
  return model;
}

estimation::MeasurementModel position_model(double position_std) {
  detail::require(position_std > 0.0, "position_model: standard deviation must be positive");
  estimation::MeasurementModel model;
  model.h = [](const VecX& x) -> VecX {
    return x.head<2>();
  };
  model.jacobian = [](const VecX& x) -> MatX {
    MatX h = MatX::Zero(2, x.size());
    h(0, 0) = 1.0;
    h(1, 1) = 1.0;
    return h;
  };
  model.noise = Mat2::Identity() * position_std * position_std;
  return model;
}

double range_bearing_log_likelihood(const Vec3& pose, const Vec2& landmark, const Vec2& z,
                                    double range_std, double bearing_std) {
  const Vec2 pred = predict_range_bearing(pose, landmark);
  const double er = (z(0) - pred(0)) / range_std;
  const double eb = math::wrap_to_pi(z(1) - pred(1)) / bearing_std;
  return -0.5 * (er * er + eb * eb) - std::log(2.0 * std::numbers::pi * range_std * bearing_std);
}

}  // namespace roboticscore::localization
