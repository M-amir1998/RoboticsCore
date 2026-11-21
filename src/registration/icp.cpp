#include "roboticscore/registration/icp.hpp"

#include <cmath>

#include <Eigen/Cholesky>

#include "roboticscore/core/error.hpp"
#include "roboticscore/spatial/kd_tree.hpp"

namespace roboticscore::registration {
namespace {

struct Pairs {
  std::vector<Vec2> src;  // transformed source points
  std::vector<Vec2> dst;  // matched target points
  std::vector<std::size_t> dst_ids;
};

// Point-to-line increment: minimise sum (n . (R(dtheta) q + d - t))^2 linearised at zero.
// Returns nullopt-equivalent identity if the system is degenerate.
geometry::SE2 point_to_line_increment(const Pairs& pairs, const std::vector<Vec2>& normals) {
  Mat3 ata = Mat3::Zero();
  Vec3 atb = Vec3::Zero();
  for (std::size_t i = 0; i < pairs.src.size(); ++i) {
    const Vec2& n = normals[pairs.dst_ids[i]];
    if (n.squaredNorm() == 0.0) {
      continue;
    }
    const Vec2& q = pairs.src[i];
    const Vec3 a(n.x(), n.y(), n.y() * q.x() - n.x() * q.y());
    const double r = n.dot(q - pairs.dst[i]);
    ata += a * a.transpose();
    atb -= a * r;
  }
  // Tiny Tikhonov term keeps the system solvable when the geometry leaves a direction
  // unconstrained (e.g. a single straight wall); it does not bias well-posed problems.
  ata += 1e-9 * (ata.trace() + 1.0) * Mat3::Identity();
  const Vec3 x = ata.ldlt().solve(atb);
  return {geometry::SO2(x(2)), Vec2(x(0), x(1))};
}

}  // namespace

geometry::SE2 best_fit_transform(std::span<const Vec2> a, std::span<const Vec2> b) {
  detail::require(a.size() == b.size(), "best_fit_transform: point sets must have equal size");
  detail::require(!a.empty(), "best_fit_transform: point sets must not be empty");
  Vec2 ca = Vec2::Zero();
  Vec2 cb = Vec2::Zero();
  for (std::size_t i = 0; i < a.size(); ++i) {
    ca += a[i];
    cb += b[i];
  }
  ca /= static_cast<double>(a.size());
  cb /= static_cast<double>(b.size());
  // Rotation maximising sum b_i' . R a_i': theta = atan2(sum a x b, sum a . b).
  double dot = 0.0;
  double cross = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const Vec2 pa = a[i] - ca;
    const Vec2 pb = b[i] - cb;
    dot += pa.dot(pb);
    cross += pa.x() * pb.y() - pa.y() * pb.x();
  }
  const geometry::SO2 r(std::atan2(cross, dot));
  return {r, cb - r * ca};
}

std::vector<Vec2> estimate_normals(std::span<const Vec2> points, int k) {
  detail::require(k >= 2, "estimate_normals: k must be at least 2");
  const spatial::KdTree2D tree(points);
  std::vector<Vec2> normals(points.size(), Vec2::Zero());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto nbrs = tree.k_nearest(points[i], static_cast<std::size_t>(k));
    if (nbrs.size() < 2) {
      continue;
    }
    Vec2 mean = Vec2::Zero();
    for (const auto& n : nbrs) {
      mean += points[n.id];
    }
    mean /= static_cast<double>(nbrs.size());
    Mat2 cov = Mat2::Zero();
    for (const auto& n : nbrs) {
      const Vec2 d = points[n.id] - mean;
      cov += d * d.transpose();
    }
    // Smallest-eigenvalue eigenvector of the 2x2 covariance, in closed form: the principal
    // direction has angle 0.5 * atan2(2 cxy, cxx - cyy); the normal is perpendicular to it.
    const double cxx = cov(0, 0);
    const double cyy = cov(1, 1);
    const double cxy = cov(0, 1);
    if (cxx + cyy <= 1e-24) {
      continue;  // all neighbours coincide
    }
    const double phi = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
    normals[i] = Vec2(-std::sin(phi), std::cos(phi));
  }
  return normals;
}

IcpResult icp_2d(std::span<const Vec2> source, std::span<const Vec2> target,
                 const IcpParams& params, const geometry::SE2& initial_guess) {
  detail::require(!source.empty() && !target.empty(), "icp_2d: point clouds must not be empty");
  detail::require(params.max_iterations > 0, "icp_2d: max_iterations must be positive");
  detail::require(params.tolerance > 0.0, "icp_2d: tolerance must be positive");
  detail::require(params.max_correspondence_distance > 0.0,
                  "icp_2d: max_correspondence_distance must be positive");

  const spatial::KdTree2D tree(target);
  std::vector<Vec2> normals;
  if (params.method == IcpMethod::kPointToLine) {
    normals = estimate_normals(target, params.normal_neighbors);
  }
  const double max_d2 = params.max_correspondence_distance * params.max_correspondence_distance;
  IcpResult result;
  result.transform = initial_guess;
  if (params.record_history) {
    result.history.push_back(initial_guess);
  }

  Pairs pairs;
  pairs.src.reserve(source.size());
  pairs.dst.reserve(source.size());
  pairs.dst_ids.reserve(source.size());
  for (int it = 0; it < params.max_iterations; ++it) {
    pairs.src.clear();
    pairs.dst.clear();
    pairs.dst_ids.clear();
    for (const Vec2& p : source) {
      const Vec2 q = result.transform * p;
      const auto nn = tree.nearest(q);
      if (nn && nn->distance_squared <= max_d2) {
        pairs.src.push_back(q);
        pairs.dst.push_back(target[nn->id]);
        pairs.dst_ids.push_back(nn->id);
      }
    }
    result.iterations = it + 1;
    result.correspondences = pairs.src.size();
    if (pairs.src.size() < params.min_correspondences) {
      result.status = IcpStatus::kTooFewCorrespondences;
      return result;
    }
    const geometry::SE2 delta = params.method == IcpMethod::kPointToLine
                                    ? point_to_line_increment(pairs, normals)
                                    : best_fit_transform(pairs.src, pairs.dst);
    result.transform = delta * result.transform;
    if (params.record_history) {
      result.history.push_back(result.transform);
    }
    double ss = 0.0;
    for (std::size_t i = 0; i < pairs.src.size(); ++i) {
      const Vec2 moved = delta * pairs.src[i];
      const double e = params.method == IcpMethod::kPointToLine
                           ? normals[pairs.dst_ids[i]].dot(moved - pairs.dst[i])
                           : (moved - pairs.dst[i]).norm();
      ss += e * e;
    }
    result.rmse = std::sqrt(ss / static_cast<double>(pairs.src.size()));
    if (std::abs(delta.theta()) < params.tolerance
        && delta.translation().norm() < params.tolerance) {
      result.status = IcpStatus::kConverged;
      return result;
    }
  }
  result.status = IcpStatus::kMaxIterations;
  return result;
}

}  // namespace roboticscore::registration
