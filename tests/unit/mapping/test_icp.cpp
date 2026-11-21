#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"
#include "roboticscore/registration/icp.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::registration {
namespace {

using geometry::SE2;

// An asymmetric "room" outline so the alignment is unique.
std::vector<Vec2> make_scene(math::Random& rng, double noise) {
  std::vector<Vec2> pts;
  const auto add_segment = [&](const Vec2& a, const Vec2& b, int n) {
    for (int i = 0; i < n; ++i) {
      const Vec2 p = a + (b - a) * (i / static_cast<double>(n));
      pts.push_back(p + Vec2(rng.normal(0, noise), rng.normal(0, noise)));
    }
  };
  add_segment({0, 0}, {6, 0}, 60);
  add_segment({6, 0}, {6, 3}, 30);
  add_segment({6, 3}, {3, 4.5}, 30);
  add_segment({3, 4.5}, {0, 4}, 30);
  add_segment({0, 4}, {0, 0}, 40);
  add_segment({2, 1}, {3, 2}, 15);  // interior feature
  return pts;
}

std::vector<Vec2> transform_all(const SE2& t, const std::vector<Vec2>& pts) {
  std::vector<Vec2> out;
  out.reserve(pts.size());
  for (const Vec2& p : pts) {
    out.push_back(t * p);
  }
  return out;
}

TEST(BestFitTransform, RecoversExactTransform) {
  math::Random rng(1);
  const auto a = make_scene(rng, 0.0);
  const SE2 t(1.5, -0.7, 0.4);
  const auto b = transform_all(t, a);
  const SE2 est = best_fit_transform(a, b);
  EXPECT_MATRIX_NEAR(est.matrix(), t.matrix(), 1e-12);
}

TEST(BestFitTransform, DegenerateInputs) {
  const std::vector<Vec2> one = {{1, 1}};
  const std::vector<Vec2> moved = {{3, 0}};
  const SE2 t = best_fit_transform(one, moved);  // pure translation for a single pair
  EXPECT_MATRIX_NEAR(t.vector(), Vec3(2, -1, 0), 1e-15);
  const std::vector<Vec2> empty;
  EXPECT_THROW((void)best_fit_transform(empty, empty), std::invalid_argument);
  EXPECT_THROW((void)best_fit_transform(one, std::vector<Vec2>{{1, 1}, {2, 2}}),
               std::invalid_argument);
}

TEST(Icp, PointToPointConvergesFromSmallMisalignment) {
  math::Random rng(2);
  const auto target = make_scene(rng, 0.0);
  // With identically sampled clouds the point-to-point basin is narrow (about 0.5 deg here);
  // see PointToPointLocalMinimumIsDocumentedBehaviour.
  const SE2 truth(0.05, -0.03, math::deg2rad(0.5));
  // source = truth^-1 * target, so aligning source onto target must recover `truth`.
  const auto source = transform_all(truth.inverse(), target);
  IcpParams params;
  params.max_iterations = 200;
  params.record_history = true;
  const IcpResult r = icp_2d(source, target, params);
  ASSERT_TRUE(r.converged());
  EXPECT_MATRIX_NEAR(r.transform.matrix(), truth.matrix(), 1e-6);
  EXPECT_LT(r.rmse, 1e-6);
  EXPECT_EQ(r.history.size(), static_cast<std::size_t>(r.iterations) + 1);
}

TEST(Icp, PointToLineConvergesFromLargerMisalignment) {
  math::Random rng(2);
  const auto target = make_scene(rng, 0.0);
  const SE2 truth(0.4, -0.3, math::deg2rad(20.0));
  const auto source = transform_all(truth.inverse(), target);
  IcpParams params;
  params.method = IcpMethod::kPointToLine;
  const IcpResult r = icp_2d(source, target, params);
  ASSERT_TRUE(r.converged());
  EXPECT_MATRIX_NEAR(r.transform.matrix(), truth.matrix(), 1e-6);
  EXPECT_LT(r.iterations, 30);
}

TEST(Icp, PointToPointLocalMinimumIsDocumentedBehaviour) {
  // Regression test for the failure mode described in icp.hpp: from 20 degrees the
  // point-to-point metric settles where sampled wall points sit between their neighbours.
  math::Random rng(2);
  const auto target = make_scene(rng, 0.0);
  const SE2 truth(0.4, -0.3, math::deg2rad(20.0));
  const auto source = transform_all(truth.inverse(), target);
  const IcpResult r = icp_2d(source, target, IcpParams{});
  EXPECT_GT(std::abs(math::angle_difference(r.transform.theta(), truth.theta())),
            math::deg2rad(0.5));
  EXPECT_GT(r.rmse, 0.01);
}

TEST(Icp, NoisyPartialOverlapWithOutlierRejection) {
  math::Random rng(3);
  const auto target = make_scene(rng, 0.01);
  const SE2 truth(0.2, 0.1, math::deg2rad(-8.0));
  auto source = transform_all(truth.inverse(), make_scene(rng, 0.01));
  source.resize(source.size() * 2 / 3);  // partial overlap
  for (int i = 0; i < 20; ++i) {
    source.emplace_back(rng.uniform(10, 20), rng.uniform(10, 20));  // spurious points
  }
  IcpParams params;
  params.method = IcpMethod::kPointToLine;
  params.max_correspondence_distance = 0.5;
  const IcpResult r = icp_2d(source, target, params);
  ASSERT_TRUE(r.converged());
  EXPECT_LT((r.transform.translation() - truth.translation()).norm(), 0.02);
  EXPECT_LT(std::abs(math::angle_difference(r.transform.theta(), truth.theta())), 0.005);
}

TEST(Icp, NormalsArePerpendicularToWalls) {
  std::vector<Vec2> line;
  for (int i = 0; i < 20; ++i) {
    line.emplace_back(0.1 * i, 2.0);
  }
  for (const Vec2& n : estimate_normals(line, 5)) {
    EXPECT_NEAR(std::abs(n.y()), 1.0, 1e-12);
  }
  EXPECT_THROW((void)estimate_normals(line, 1), std::invalid_argument);
}

TEST(Icp, ReportsTooFewCorrespondences) {
  const std::vector<Vec2> target = {{0, 0}, {1, 0}, {0, 1}};
  const std::vector<Vec2> source = {{100, 100}, {101, 100}, {100, 101}};
  IcpParams params;
  params.max_correspondence_distance = 1.0;
  const IcpResult r = icp_2d(source, target, params);
  EXPECT_EQ(r.status, IcpStatus::kTooFewCorrespondences);
  EXPECT_THROW((void)icp_2d({}, target), std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::registration
