#pragma once

/// @file pid.hpp
/// @brief Discrete PID controller with output limits, anti-windup and a filtered derivative.
///
///   u = Kp e + I + D,   e = setpoint - measurement,   I = Ki * integral(e)
///
/// Design choices (Astrom & Murray, "Feedback Systems", 2nd ed., ch. 11):
///   - The derivative acts on the measurement (no derivative kick on setpoint steps) and is
///     low-pass filtered with time constant tau: D_k = a D_{k-1} - Kd (1 - a)(y_k - y_{k-1})/dt
///     with a = tau / (tau + dt). tau = 0 disables the filter.
///   - The integral stores Ki * integral(e) so gain changes do not bump the output.
///   - Anti-windup by conditional integration (clamping) or by back-calculation with
///     tracking gain Kt: dI/dt = Ki e + Kt (u_sat - u).

#include <limits>

namespace roboticscore::control {

enum class AntiWindup { kNone, kClamping, kBackCalculation };

/// PID parameters.
struct PidConfig {
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.0;
  double output_min = -std::numeric_limits<double>::infinity();
  double output_max = std::numeric_limits<double>::infinity();
  double derivative_filter_tau = 0.0;  ///< [s]
  AntiWindup anti_windup = AntiWindup::kClamping;
  double back_calculation_gain = 1.0;  ///< Kt [1/s]
};

/// Per-term breakdown of the last update (for logging and tests).
struct PidTerms {
  double proportional = 0.0;
  double integral = 0.0;
  double derivative = 0.0;
  double unsaturated = 0.0;
  double output = 0.0;
};

/// Stateful PID controller.
class PidController {
 public:
  /// @throws std::invalid_argument for negative gains, output_min > output_max, negative tau
  ///         or negative back-calculation gain.
  explicit PidController(const PidConfig& config);

  /// Computes the control output for one sample period @p dt.
  /// @throws std::invalid_argument if dt is not positive.
  double update(double setpoint, double measurement, double dt);

  /// Clears the integral and derivative state.
  void reset();

  [[nodiscard]] const PidTerms& last_terms() const noexcept { return terms_; }
  [[nodiscard]] const PidConfig& config() const noexcept { return config_; }

 private:
  PidConfig config_;
  double integral_ = 0.0;
  double derivative_ = 0.0;
  double previous_measurement_ = 0.0;
  bool has_previous_ = false;
  PidTerms terms_;
};

}  // namespace roboticscore::control
