#pragma once

/// @file pose_graph.hpp
/// @brief 2D pose-graph SLAM back end with sparse Gauss-Newton / Levenberg-Marquardt.
///
/// Nodes are robot poses x_i = (x, y, theta); an edge (i, j) stores a measured relative pose
/// Z_ij and its information matrix Omega_ij. Following Grisetti, Kuemmerle, Stachniss &
/// Burgard, "A Tutorial on Graph-Based SLAM" (IEEE ITS Magazine, 2010), the edge error is
///   e_ij = [ R_z^T (R_i^T (t_j - t_i) - t_z) ;  wrap(theta_j - theta_i - theta_z) ]
/// with analytic Jacobians, and each iteration solves the sparse normal equations
///   H dx = -b,  H = sum J^T Omega J,  b = sum J^T Omega e
/// with a sparse LDLT factorisation. The gauge freedom is removed by holding fixed nodes
/// (node 0 by default) constant. Cost: O(nnz) to build, sparse factorisation to solve.
///
/// For large problems or robust kernels use a dedicated back end (g2o, GTSAM, Ceres).

#include <cstddef>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::slam {

/// Relative-pose constraint.
struct PoseGraphEdge {
  std::size_t from = 0;
  std::size_t to = 0;
  geometry::Pose2 measurement;  ///< Z_ij = X_i^-1 X_j as measured.
  Mat3 information = Mat3::Identity();
};

/// Optimisation settings.
struct PoseGraphOptions {
  enum class Method { kGaussNewton, kLevenbergMarquardt };
  Method method = Method::kLevenbergMarquardt;
  int max_iterations = 50;
  double relative_tolerance = 1e-10;  ///< Stop when chi2 decreases by less than this fraction.
  double initial_lambda = 1e-4;
  bool record_history = false;  ///< Keep all poses after every iteration.
};

/// Optimisation outcome.
struct PoseGraphResult {
  bool converged = false;
  int iterations = 0;
  double initial_chi2 = 0.0;
  double final_chi2 = 0.0;
  std::vector<double> chi2_history;
  std::vector<std::vector<geometry::Pose2>> pose_history;  ///< If record_history is set.
};

/// Error and Jacobians of one edge (exposed for testing).
struct EdgeLinearization {
  Vec3 error = Vec3::Zero();
  Mat3 jacobian_from = Mat3::Zero();
  Mat3 jacobian_to = Mat3::Zero();
};

[[nodiscard]] EdgeLinearization linearize_edge(const geometry::Pose2& xi, const geometry::Pose2& xj,
                                               const geometry::Pose2& z);

/// Pose graph over planar poses.
class PoseGraph2D {
 public:
  /// Adds a node with initial estimate @p pose; returns its index. Node 0 is fixed by default.
  std::size_t add_node(const geometry::Pose2& pose);

  /// @throws std::invalid_argument for unknown nodes, self-loops or an information matrix that
  ///         is not symmetric positive definite.
  void add_edge(const PoseGraphEdge& edge);

  /// Marks a node as fixed (gauge anchor). @throws std::out_of_range for unknown nodes.
  void set_fixed(std::size_t node, bool fixed = true);

  /// Optimises all free poses.
  /// @throws std::invalid_argument if no node is fixed (the problem would be singular).
  PoseGraphResult optimize(const PoseGraphOptions& options = {});

  /// Sum of e^T Omega e over all edges.
  [[nodiscard]] double chi2() const;

  [[nodiscard]] const std::vector<geometry::Pose2>& poses() const noexcept { return poses_; }
  [[nodiscard]] const std::vector<PoseGraphEdge>& edges() const noexcept { return edges_; }
  void set_pose(std::size_t node, const geometry::Pose2& pose) { poses_.at(node) = pose; }

 private:
  std::vector<geometry::Pose2> poses_;
  std::vector<PoseGraphEdge> edges_;
  std::vector<bool> fixed_;
};

}  // namespace roboticscore::slam
