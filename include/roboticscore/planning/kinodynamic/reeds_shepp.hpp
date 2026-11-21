#pragma once

/// @file reeds_shepp.hpp
/// @brief Shortest paths for a car that can drive forwards and backwards with a bounded
///        turning radius (Reeds & Shepp, Pacific J. Math. 145(2), 1990).
///
/// The optimal path is one of 48 words built from left/right arcs (L, R) and straight
/// segments (S), with at most two direction reversals. The closed-form solutions of the five
/// families CSC, CCC, CCCC, CCSC and CCSCC are evaluated together with their time-flip,
/// reflection and backwards symmetries; every candidate is verified by integrating it before
/// it can be selected.
///
/// Reeds-Shepp length is an admissible heuristic for Hybrid A* (it ignores obstacles but
/// respects the kinematics).

#include <array>
#include <optional>
#include <vector>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::planning {

/// Segment type of a Reeds-Shepp word.
enum class RsSegment : std::uint8_t { kLeft, kStraight, kRight };

/// One Reeds-Shepp path: up to five segments with signed lengths (negative = reverse).
/// Lengths are in units of the turning radius for arcs (radians) and for straights.
struct ReedsSheppPath {
  std::array<RsSegment, 5> types{};
  std::array<double, 5> lengths{};  ///< Unused trailing entries are zero.
  double radius = 1.0;              ///< Turning radius used to scale the path.

  /// Total path length in metres.
  [[nodiscard]] double length() const;
  /// Number of direction reversals.
  [[nodiscard]] int reversals() const;
};

/// A pose sampled along a path together with its driving direction.
struct DirectedPose {
  geometry::Pose2 pose;
  bool forward = true;
  double curvature = 0.0;  ///< Signed path curvature [1/m].
};

/// Shortest Reeds-Shepp path from @p start to @p goal with turning radius @p radius.
/// @throws std::invalid_argument if radius is not positive.
[[nodiscard]] ReedsSheppPath reeds_shepp_shortest(const geometry::Pose2& start,
                                                  const geometry::Pose2& goal, double radius);

/// All valid candidate words (useful for tests and visualisation).
[[nodiscard]] std::vector<ReedsSheppPath> reeds_shepp_candidates(const geometry::Pose2& start,
                                                                 const geometry::Pose2& goal,
                                                                 double radius);

/// Samples the path every @p step metres (plus segment ends and the final pose).
/// @throws std::invalid_argument if step is not positive.
[[nodiscard]] std::vector<DirectedPose> sample_reeds_shepp(const geometry::Pose2& start,
                                                           const ReedsSheppPath& path, double step);

}  // namespace roboticscore::planning
