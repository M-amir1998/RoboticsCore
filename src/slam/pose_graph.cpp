#include "roboticscore/slam/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "roboticscore/core/error.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/linear_algebra.hpp"

namespace roboticscore::slam {
namespace {

Mat2 rot(double th) {
  Mat2 r;
  r << std::cos(th), -std::sin(th), std::sin(th), std::cos(th);
  return r;
}

}  // namespace

EdgeLinearization linearize_edge(const geometry::Pose2& xi, const geometry::Pose2& xj,
                                 const geometry::Pose2& z) {
  const Mat2 ri = rot(xi.theta());
  const Mat2 rz = rot(z.theta());
  const Vec2 dt = xj.translation() - xi.translation();
  // dR_i^T / dtheta_i
  Mat2 dri_t;
  dri_t << -std::sin(xi.theta()), std::cos(xi.theta()), -std::cos(xi.theta()),
      -std::sin(xi.theta());
  EdgeLinearization lin;
  lin.error.head<2>() = rz.transpose() * (ri.transpose() * dt - z.translation());
  lin.error(2) = math::wrap_to_pi(xj.theta() - xi.theta() - z.theta());
  lin.jacobian_from.topLeftCorner<2, 2>() = -rz.transpose() * ri.transpose();
  lin.jacobian_from.topRightCorner<2, 1>() = rz.transpose() * dri_t * dt;
  lin.jacobian_from(2, 2) = -1.0;
  lin.jacobian_to.topLeftCorner<2, 2>() = rz.transpose() * ri.transpose();
  lin.jacobian_to(2, 2) = 1.0;
  return lin;
}

std::size_t PoseGraph2D::add_node(const geometry::Pose2& pose) {
  poses_.push_back(pose);
  fixed_.push_back(poses_.size() == 1);
  return poses_.size() - 1;
}

void PoseGraph2D::add_edge(const PoseGraphEdge& edge) {
  detail::require(edge.from < poses_.size() && edge.to < poses_.size(),
                  "PoseGraph2D::add_edge: unknown node");
  detail::require(edge.from != edge.to, "PoseGraph2D::add_edge: self-loops are not allowed");
  detail::require(math::is_positive_definite(edge.information),
                  "PoseGraph2D::add_edge: information must be symmetric positive definite");
  edges_.push_back(edge);
}

void PoseGraph2D::set_fixed(std::size_t node, bool fixed) {
  if (node >= poses_.size()) {
    throw std::out_of_range("PoseGraph2D::set_fixed: unknown node");
  }
  fixed_[node] = fixed;
}

double PoseGraph2D::chi2() const {
  double sum = 0.0;
  for (const auto& e : edges_) {
    const Vec3 err = linearize_edge(poses_[e.from], poses_[e.to], e.measurement).error;
    sum += err.dot(e.information * err);
  }
  return sum;
}

PoseGraphResult PoseGraph2D::optimize(const PoseGraphOptions& options) {
  detail::require(std::find(fixed_.begin(), fixed_.end(), true) != fixed_.end(),
                  "PoseGraph2D::optimize: at least one node must be fixed");
  // Variable index of each free node (-1 for fixed nodes).
  std::vector<Index> var(poses_.size(), -1);
  Index n_free = 0;
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    if (!fixed_[i]) var[i] = 3 * n_free++;
  }
  const Index dim = 3 * n_free;
  PoseGraphResult result;
  result.initial_chi2 = result.final_chi2 = chi2();
  result.chi2_history.push_back(result.final_chi2);
  if (options.record_history) result.pose_history.push_back(poses_);
  if (dim == 0) {
    result.converged = true;
    return result;
  }

  double lambda = -1.0;
  double nu = 2.0;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  bool pattern_analyzed = false;
  for (int it = 0; it < options.max_iterations; ++it) {
    result.iterations = it + 1;
    // Assemble the sparse normal equations.
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(edges_.size() * 36);
    VecX b = VecX::Zero(dim);
    for (const auto& e : edges_) {
      const EdgeLinearization lin = linearize_edge(poses_[e.from], poses_[e.to], e.measurement);
      const Index vi = var[e.from];
      const Index vj = var[e.to];
      const Mat3 ai = lin.jacobian_from;
      const Mat3 bj = lin.jacobian_to;
      const auto add_block = [&](Index r, Index c, const Mat3& m) {
        for (Index p = 0; p < 3; ++p) {
          for (Index q = 0; q < 3; ++q) {
            triplets.emplace_back(static_cast<int>(r + p), static_cast<int>(c + q), m(p, q));
          }
        }
      };
      if (vi >= 0) {
        add_block(vi, vi, ai.transpose() * e.information * ai);
        b.segment<3>(vi) += ai.transpose() * e.information * lin.error;
      }
      if (vj >= 0) {
        add_block(vj, vj, bj.transpose() * e.information * bj);
        b.segment<3>(vj) += bj.transpose() * e.information * lin.error;
      }
      if (vi >= 0 && vj >= 0) {
        const Mat3 hij = ai.transpose() * e.information * bj;
        add_block(vi, vj, hij);
        add_block(vj, vi, hij.transpose());
      }
    }
    Eigen::SparseMatrix<double> h(dim, dim);
    h.setFromTriplets(triplets.begin(), triplets.end());
    const VecX diag = h.diagonal();
    if (options.method == PoseGraphOptions::Method::kLevenbergMarquardt && lambda < 0.0) {
      lambda = options.initial_lambda * std::max(diag.maxCoeff(), 1e-12);
    }

    const std::vector<geometry::Pose2> backup = poses_;
    const double old_chi2 = result.final_chi2;
    bool accepted = false;
    for (int attempt = 0; attempt < 20 && !accepted; ++attempt) {
      Eigen::SparseMatrix<double> damped = h;
      if (options.method == PoseGraphOptions::Method::kLevenbergMarquardt) {
        for (Index k = 0; k < dim; ++k) {
          damped.coeffRef(k, k) += lambda * std::max(diag(k), 1e-12);
        }
      } else {
        // Tiny regulariser for numerically singular but well-posed systems.
        for (Index k = 0; k < dim; ++k)
          damped.coeffRef(k, k) += 1e-12;
      }
      if (!pattern_analyzed) {
        solver.analyzePattern(damped);
        pattern_analyzed = true;
      }
      solver.factorize(damped);
      if (solver.info() != Eigen::Success) {
        result.converged = false;
        return result;
      }
      const VecX dx = solver.solve(-b);
      for (std::size_t i = 0; i < poses_.size(); ++i) {
        if (var[i] < 0) continue;
        const Vec3 d = dx.segment<3>(var[i]);
        poses_[i] = geometry::Pose2(backup[i].x() + d(0), backup[i].y() + d(1),
                                    math::wrap_to_pi(backup[i].theta() + d(2)));
      }
      const double new_chi2 = chi2();
      if (options.method == PoseGraphOptions::Method::kGaussNewton) {
        accepted = true;
        result.final_chi2 = new_chi2;
        break;
      }
      // LM gain ratio: predicted reduction of the quadratic model (Nielsen update).
      const double predicted = -2.0 * b.dot(dx) - dx.dot(h * dx);
      const double rho = predicted > 0.0 ? (old_chi2 - new_chi2) / predicted : -1.0;
      if (rho > 0.0) {
        accepted = true;
        result.final_chi2 = new_chi2;
        lambda *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * rho - 1.0, 3));
        nu = 2.0;
      } else {
        poses_ = backup;
        lambda *= nu;
        nu *= 2.0;
      }
    }
    if (!accepted) {
      result.converged = old_chi2 <= 1e-12;
      return result;
    }
    result.chi2_history.push_back(result.final_chi2);
    if (options.record_history) result.pose_history.push_back(poses_);
    if (std::abs(old_chi2 - result.final_chi2)
        <= options.relative_tolerance * std::max(old_chi2, 1e-300)) {
      result.converged = true;
      return result;
    }
  }
  result.converged = false;
  return result;
}

}  // namespace roboticscore::slam
