#include "roboticscore/control/pid.hpp"

#include <algorithm>

#include "roboticscore/core/error.hpp"

namespace roboticscore::control {

PidController::PidController(const PidConfig& config) : config_(config) {
  detail::require(config.kp >= 0.0 && config.ki >= 0.0 && config.kd >= 0.0,
                  "PidController: gains must be non-negative");
  detail::require(config.output_min <= config.output_max,
                  "PidController: output_min must not exceed output_max");
  detail::require(config.derivative_filter_tau >= 0.0,
                  "PidController: derivative filter time constant must be non-negative");
  detail::require(config.back_calculation_gain >= 0.0,
                  "PidController: back-calculation gain must be non-negative");
}

void PidController::reset() {
  integral_ = 0.0;
  derivative_ = 0.0;
  has_previous_ = false;
  terms_ = {};
}

double PidController::update(double setpoint, double measurement, double dt) {
  detail::require(dt > 0.0, "PidController::update: dt must be positive");
  const double error = setpoint - measurement;
  const double p = config_.kp * error;

  if (has_previous_ && config_.kd > 0.0) {
    const double raw = -config_.kd * (measurement - previous_measurement_) / dt;
    const double a = config_.derivative_filter_tau / (config_.derivative_filter_tau + dt);
    derivative_ = a * derivative_ + (1.0 - a) * raw;
  }
  previous_measurement_ = measurement;
  has_previous_ = true;

  const auto saturate = [&](double u) {
    return std::clamp(u, config_.output_min, config_.output_max);
  };

  double integral = integral_ + config_.ki * error * dt;
  double unsaturated = p + integral + derivative_;
  double output = saturate(unsaturated);
  switch (config_.anti_windup) {
    case AntiWindup::kNone:
      break;
    case AntiWindup::kClamping:
      // Freeze the integrator when saturated and the error would push further into the limit.
      if (output != unsaturated && error * (unsaturated - output) > 0.0) {
        integral = integral_;
        unsaturated = p + integral + derivative_;
        output = saturate(unsaturated);
      }
      break;
    case AntiWindup::kBackCalculation:
      integral += config_.back_calculation_gain * (output - unsaturated) * dt;
      break;
  }
  integral_ = integral;
  terms_ = {p, integral_, derivative_, unsaturated, output};
  return output;
}

}  // namespace roboticscore::control
