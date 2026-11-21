#pragma once

/// @file kd_tree.hpp
/// @brief 2D k-d tree for nearest-neighbour, k-nearest and radius queries.
///
/// Supports both a balanced bulk build (median splits, O(n log n)) and incremental insertion
/// (O(depth)), which sampling-based planners need as their trees grow. Incremental insertion
/// does not rebalance; for random insertion orders the expected depth is O(log n), while
/// adversarial orders (e.g. sorted points) degrade towards O(n). Call rebuild() to rebalance.
///
/// Determinism: ties in distance are broken by the smaller point id, so query results do not
/// depend on tree shape.
///
/// Complexity (balanced): nearest/k-nearest O(log n) expected, radius O(log n + m).

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "roboticscore/core/types.hpp"

namespace roboticscore::spatial {

/// Result of a neighbour query.
struct Neighbor {
  std::size_t id = 0;             ///< Insertion index of the point.
  double distance_squared = 0.0;  ///< Squared Euclidean distance to the query.
};

/// k-d tree over 2D points; point ids are their insertion indices.
class KdTree2D {
 public:
  KdTree2D() = default;

  /// Balanced build over @p points; ids are 0..n-1 in the given order.
  explicit KdTree2D(std::span<const Vec2> points);

  /// Inserts a point and returns its id.
  std::size_t insert(const Vec2& p);

  /// Rebuilds a balanced tree over the current points (ids are preserved).
  void rebuild();

  void clear();

  [[nodiscard]] std::size_t size() const noexcept { return points_.size(); }
  [[nodiscard]] bool empty() const noexcept { return points_.empty(); }
  [[nodiscard]] const Vec2& point(std::size_t id) const { return points_.at(id); }
  [[nodiscard]] const std::vector<Vec2>& points() const noexcept { return points_; }

  /// Nearest point to @p q, or nullopt if the tree is empty.
  [[nodiscard]] std::optional<Neighbor> nearest(const Vec2& q) const;

  /// Up to @p k nearest points sorted by increasing distance (ties by id).
  [[nodiscard]] std::vector<Neighbor> k_nearest(const Vec2& q, std::size_t k) const;

  /// All points with distance <= @p radius, sorted by increasing distance (ties by id).
  /// @throws std::invalid_argument if radius is negative.
  [[nodiscard]] std::vector<Neighbor> radius_search(const Vec2& q, double radius) const;

  /// Depth of the tree (0 for empty); exposed for tests and diagnostics.
  [[nodiscard]] std::size_t depth() const;

 private:
  static constexpr int kNone = -1;

  struct Node {
    int left = kNone;
    int right = kNone;
    int axis = 0;
  };

  int build(std::vector<int>& ids, std::size_t begin, std::size_t end, int depth);

  std::vector<Vec2> points_;
  std::vector<Node> nodes_;  // nodes_[i] is the node holding points_[i]
  int root_ = kNone;
};

}  // namespace roboticscore::spatial
