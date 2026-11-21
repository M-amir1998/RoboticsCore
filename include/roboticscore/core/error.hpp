#pragma once

/// @file error.hpp
/// @brief Library-wide error-handling policy (see docs/adr/0002-error-handling.md).
///
/// - Contract violations (wrong dimensions, non-positive step sizes, start cell outside the
///   map, ...) throw std::invalid_argument. They indicate a bug in the caller.
/// - Numerical breakdowns the caller could not reasonably prevent (a covariance that lost
///   positive definiteness, a singular normal-equation system) throw NumericalError.
/// - Expected algorithmic outcomes (no path exists, solver hit its iteration limit) are
///   reported through result structs with an explicit status; they never throw.

#include <stdexcept>
#include <string>

namespace roboticscore {

/// Raised when a computation fails for numerical reasons, e.g. loss of positive
/// definiteness or a singular linear system inside an iterative method.
class NumericalError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

namespace detail {

/// Throws std::invalid_argument with @p message when @p condition is false.
inline void require(bool condition, const char* message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

/// Throws std::invalid_argument with @p message when @p condition is false.
inline void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

}  // namespace detail
}  // namespace roboticscore
