#pragma once

/// @file icp.hpp
/// @brief 2D Iterative Closest Point: point-to-point (Besl & McKay, 1992) and point-to-line
///        (Chen & Medioni, 1992; the 2D variant used for LiDAR scan matching).
///
/// Each iteration (1) transforms the source cloud with the current estimate, (2) pairs every
/// source point with its nearest target point (k-d tree, O(log m) per query), (3) rejects
/// pairs farther than max_correspondence_distance, and (4) solves for an increment:
///   - point-to-point: closed-form rigid alignment of the pairs;
///   - point-to-line:  linearised least squares on the residuals n_j . (T p_i - q_j), where
///                     n_j is the target surface normal estimated by PCA of its neighbours.
/// Point-to-point ICP converges slowly along planar structure and can stall in local minima
/// where sampled points sit between their discrete neighbours; point-to-line lets points slide
/// along walls and typically converges in a few iterations. Both need an initial guess inside
/// the basin of attraction.
///
/// Complexity per iteration: O(n log m) for n source and m target points.

#include <span>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::registration {

/// Error metric minimised by ICP.
enum class IcpMethod { kPointToPoint, kPointToLine };

/// ICP configuration.
struct IcpParams {
  IcpMethod method = IcpMethod::kPointToPoint;
  /// Neighbours (including the point itself) used to estimate target normals.
  int normal_neighbors = 5;
  int max_iterations = 50;
  /// Converged when the incremental rotation and translation both fall below this value.
  double tolerance = 1e-6;
  /// Pairs farther apart than this are ignored (handles partial overlap).
  double max_correspondence_distance = 1e9;
  /// Minimum number of accepted pairs; fewer aborts with kTooFewCorrespondences.
  std::size_t min_correspondences = 3;
  /// Store the estimate after every iteration (for visualisation and diagnostics).
  bool record_history = false;
};

enum class IcpStatus { kConverged, kMaxIterations, kTooFewCorrespondences };

/// ICP outcome.
struct IcpResult {
  geometry::SE2 transform;  ///< Maps source points into the target frame.
  IcpStatus status = IcpStatus::kMaxIterations;
  int iterations = 0;
  double rmse = 0.0;  ///< RMS distance of accepted pairs after the final iteration.
  std::size_t correspondences = 0;
  std::vector<geometry::SE2> history;  ///< Filled when record_history is set.

  [[nodiscard]] bool converged() const { return status == IcpStatus::kConverged; }
};

/// Least-squares rigid transform T minimising sum |T a_i - b_i|^2 (closed form via the 2D
/// cross-covariance; reflections cannot occur in 2D with this parameterisation).
/// With identical points the rotation is undefined and identity rotation is returned.
/// @throws std::invalid_argument if the spans differ in size or are empty.
[[nodiscard]] geometry::SE2 best_fit_transform(std::span<const Vec2> a, std::span<const Vec2> b);

/// Unit surface normals of a 2D point cloud from PCA over the @p k nearest neighbours.
/// Points whose neighbourhood is degenerate get the normal (0, 0).
/// @throws std::invalid_argument if k < 2.
[[nodiscard]] std::vector<Vec2> estimate_normals(std::span<const Vec2> points, int k = 5);

/// Aligns @p source to @p target starting from @p initial_guess.
/// @throws std::invalid_argument if either cloud is empty or parameters are invalid.
[[nodiscard]] IcpResult icp_2d(std::span<const Vec2> source, std::span<const Vec2> target,
                               const IcpParams& params = {},
                               const geometry::SE2& initial_guess = {});

}  // namespace roboticscore::registration
