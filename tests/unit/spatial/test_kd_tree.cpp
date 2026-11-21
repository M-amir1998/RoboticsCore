#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "roboticscore/math/random.hpp"
#include "roboticscore/spatial/kd_tree.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::spatial {
namespace {

std::vector<Neighbor> brute_force(const std::vector<Vec2>& pts, const Vec2& q) {
  std::vector<Neighbor> all;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    all.push_back({i, (pts[i] - q).squaredNorm()});
  }
  std::sort(all.begin(), all.end(), [](const Neighbor& a, const Neighbor& b) {
    return a.distance_squared < b.distance_squared
           || (a.distance_squared == b.distance_squared && a.id < b.id);
  });
  return all;
}

TEST(KdTree2D, EmptyTree) {
  const KdTree2D tree;
  EXPECT_FALSE(tree.nearest({0, 0}).has_value());
  EXPECT_TRUE(tree.k_nearest({0, 0}, 3).empty());
  EXPECT_TRUE(tree.radius_search({0, 0}, 1.0).empty());
  EXPECT_EQ(tree.depth(), 0U);
}

TEST(KdTree2D, DuplicatesAndTieBreaking) {
  KdTree2D tree;
  tree.insert({1, 1});
  tree.insert({1, 1});
  tree.insert({0, 0});
  const auto n = tree.nearest({1, 1});
  ASSERT_TRUE(n.has_value());
  EXPECT_EQ(n->id, 0U);  // smallest id among exact ties
  const auto k = tree.k_nearest({1, 1}, 5);
  ASSERT_EQ(k.size(), 3U);
  EXPECT_EQ(k[0].id, 0U);
  EXPECT_EQ(k[1].id, 1U);
  EXPECT_EQ(k[2].id, 2U);
  EXPECT_THROW((void)tree.radius_search({0, 0}, -1.0), std::invalid_argument);
}

TEST(KdTree2D, BalancedBuildHasLogarithmicDepth) {
  std::vector<Vec2> pts;
  for (int i = 0; i < 1024; ++i) {
    pts.emplace_back(i, 0.0);  // sorted input: worst case for incremental insertion
  }
  const KdTree2D balanced(pts);
  EXPECT_LE(balanced.depth(), 11U);
  KdTree2D incremental;
  for (const auto& p : pts) {
    incremental.insert(p);
  }
  EXPECT_EQ(incremental.depth(), 1024U);  // documented degradation
  incremental.rebuild();
  EXPECT_LE(incremental.depth(), 11U);
}

TEST(KdTree2DProperty, MatchesBruteForce) {
  math::Random rng(testing::kPropertySeed);
  for (const bool bulk : {true, false}) {
    std::vector<Vec2> pts;
    for (int i = 0; i < 600; ++i) {
      // Snap to a coarse lattice so duplicate coordinates and exact ties are common.
      pts.emplace_back(std::round(rng.uniform(0, 30)) * 0.5, std::round(rng.uniform(0, 30)) * 0.5);
    }
    KdTree2D tree;
    if (bulk) {
      tree = KdTree2D(pts);
    } else {
      for (const auto& p : pts) {
        tree.insert(p);
      }
    }
    for (int trial = 0; trial < 200; ++trial) {
      const Vec2 q(rng.uniform(-2, 17), rng.uniform(-2, 17));
      const auto expected = brute_force(pts, q);
      const auto nn = tree.nearest(q);
      ASSERT_TRUE(nn.has_value());
      ASSERT_EQ(nn->id, expected[0].id);

      const auto knn = tree.k_nearest(q, 7);
      ASSERT_EQ(knn.size(), 7U);
      for (std::size_t i = 0; i < knn.size(); ++i) {
        ASSERT_EQ(knn[i].id, expected[i].id) << "bulk=" << bulk << " trial=" << trial;
      }

      const double r = rng.uniform(0.0, 3.0);
      const auto within = tree.radius_search(q, r);
      std::size_t count = 0;
      while (count < expected.size() && expected[count].distance_squared <= r * r) {
        ++count;
      }
      ASSERT_EQ(within.size(), count);
      for (std::size_t i = 0; i < within.size(); ++i) {
        ASSERT_EQ(within[i].id, expected[i].id);
      }
    }
  }
}

}  // namespace
}  // namespace roboticscore::spatial
