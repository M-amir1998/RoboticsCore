#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "roboticscore/geometry/collision.hpp"
#include "roboticscore/geometry/environment.hpp"
#include "roboticscore/geometry/grid.hpp"
#include "roboticscore/math/angles.hpp"
#include "roboticscore/math/random.hpp"

#include "roboticscore_test/eigen_testing.hpp"

namespace roboticscore::geometry {
namespace {

Polygon unit_square() {
  return Polygon({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
}

TEST(Polygon, OrientationNormalizedToCounterClockwise) {
  const Polygon cw({{0, 0}, {0, 1}, {1, 1}, {1, 0}});
  EXPECT_GT(cw.area(), 0.0);
  EXPECT_NEAR(cw.area(), 1.0, 1e-15);
  EXPECT_MATRIX_NEAR(cw.centroid(), Vec2(0.5, 0.5), 1e-15);
  EXPECT_THROW(Polygon({{0, 0}, {1, 1}}), std::invalid_argument);
  EXPECT_THROW(Polygon({{0, 0}, {1, 1}, {2, 2}}), std::invalid_argument);
}

TEST(Polygon, RotatedRectangle) {
  const Polygon r = Polygon::rectangle({0, 0}, 4.0, 2.0, math::kHalfPi);
  EXPECT_NEAR(r.area(), 8.0, 1e-12);
  EXPECT_NEAR(r.bounding_box().size().x(), 2.0, 1e-12);
  EXPECT_NEAR(r.bounding_box().size().y(), 4.0, 1e-12);
}

TEST(Collision, PointSegmentDistance) {
  const Segment s{{0, 0}, {2, 0}};
  EXPECT_DOUBLE_EQ(point_segment_distance({1, 1}, s), 1.0);
  EXPECT_DOUBLE_EQ(point_segment_distance({3, 0}, s), 1.0);
  EXPECT_DOUBLE_EQ(point_segment_distance({-3, 4}, s), 5.0);
  // Degenerate segment behaves like a point.
  EXPECT_DOUBLE_EQ(point_segment_distance({3, 4}, {{0, 0}, {0, 0}}), 5.0);
}

TEST(Collision, SegmentIntersectionCases) {
  EXPECT_TRUE(segments_intersect({{0, 0}, {2, 2}}, {{0, 2}, {2, 0}}));   // proper crossing
  EXPECT_TRUE(segments_intersect({{0, 0}, {2, 0}}, {{1, 0}, {3, 0}}));   // collinear overlap
  EXPECT_TRUE(segments_intersect({{0, 0}, {1, 0}}, {{1, 0}, {1, 1}}));   // shared endpoint
  EXPECT_FALSE(segments_intersect({{0, 0}, {1, 0}}, {{2, 0}, {3, 0}}));  // collinear gap
  EXPECT_FALSE(segments_intersect({{0, 0}, {1, 0}}, {{0, 1}, {1, 1}}));  // parallel
  EXPECT_DOUBLE_EQ(segment_segment_distance({{0, 0}, {1, 0}}, {{0, 1}, {1, 1}}), 1.0);
}

TEST(Collision, PointInPolygonIncludingBoundaryAndConcave) {
  const Polygon sq = unit_square();
  EXPECT_TRUE(point_in_polygon({0.5, 0.5}, sq));
  EXPECT_TRUE(point_in_polygon({1.0, 0.5}, sq));  // boundary
  EXPECT_TRUE(point_in_polygon({0.0, 0.0}, sq));  // vertex
  EXPECT_FALSE(point_in_polygon({1.5, 0.5}, sq));
  // Concave "U" shape: the notch is outside.
  const Polygon u({{0, 0}, {3, 0}, {3, 3}, {2, 3}, {2, 1}, {1, 1}, {1, 3}, {0, 3}});
  EXPECT_FALSE(point_in_polygon({1.5, 2.0}, u));
  EXPECT_TRUE(point_in_polygon({0.5, 2.0}, u));
}

TEST(Collision, SignedDistances) {
  const Polygon sq = unit_square();
  EXPECT_NEAR(signed_distance(Vec2(0.5, 0.5), sq), -0.5, 1e-15);
  EXPECT_NEAR(signed_distance(Vec2(2.0, 0.5), sq), 1.0, 1e-15);
  EXPECT_NEAR(signed_distance(Vec2(3, 4), Circle{{0, 0}, 1.0}), 4.0, 1e-15);
}

TEST(Collision, SegmentPolygonAndCircle) {
  const Polygon sq = unit_square();
  EXPECT_DOUBLE_EQ(segment_polygon_distance({{-1, 0.5}, {2, 0.5}}, sq), 0.0);     // passes through
  EXPECT_DOUBLE_EQ(segment_polygon_distance({{0.2, 0.2}, {0.3, 0.3}}, sq), 0.0);  // inside
  EXPECT_NEAR(segment_polygon_distance({{-1, 2}, {2, 2}}, sq), 1.0, 1e-15);
  EXPECT_NEAR(segment_circle_distance({{-2, 2}, {2, 2}}, {{0, 0}, 1.0}), 1.0, 1e-15);
  EXPECT_DOUBLE_EQ(segment_circle_distance({{-2, 0}, {2, 0}}, {{0, 0}, 1.0}), 0.0);
}

TEST(Collision, PolygonOverlap) {
  const Polygon sq = unit_square();
  EXPECT_TRUE(polygons_intersect(sq, Polygon::rectangle({1.0, 1.0}, 1.0, 1.0)));
  EXPECT_TRUE(polygons_intersect(sq, Polygon::rectangle({0.5, 0.5}, 0.2, 0.2)));  // contained
  EXPECT_FALSE(polygons_intersect(sq, Polygon::rectangle({3.0, 3.0}, 1.0, 1.0)));
  EXPECT_TRUE(polygon_circle_intersect(sq, {{1.5, 0.5}, 0.6}));
  EXPECT_FALSE(polygon_circle_intersect(sq, {{1.5, 0.5}, 0.4}));
  EXPECT_TRUE(polygon_circle_intersect(sq, {{0.5, 0.5}, 0.01}));  // circle inside
}

TEST(Collision, RayCasting) {
  const Vec2 o(0, 0);
  const Vec2 east(1, 0);
  EXPECT_NEAR(*ray_cast(o, east, Circle{{5, 0}, 1.0}), 4.0, 1e-12);
  EXPECT_FALSE(ray_cast(o, east, Circle{{-5, 0}, 1.0}).has_value());
  EXPECT_DOUBLE_EQ(*ray_cast(o, east, Circle{{0, 0}, 1.0}), 0.0);
  EXPECT_NEAR(*ray_cast(o, east, Segment{{3, -1}, {3, 1}}), 3.0, 1e-15);
  EXPECT_FALSE(ray_cast(o, east, Segment{{3, 1}, {3, 2}}).has_value());
  EXPECT_NEAR(*ray_cast(o, east, Segment{{2, 0}, {5, 0}}), 2.0, 1e-15);  // collinear
  EXPECT_NEAR(*ray_cast(o, east, Polygon::rectangle({5, 0}, 2.0, 2.0)), 4.0, 1e-12);
}

TEST(Environment, ClearanceAndFreeSpace) {
  Environment2D env({{0, 0}, {10, 10}});
  env.add_obstacle(Circle{{5, 5}, 1.0});
  env.add_obstacle(Polygon::rectangle({2, 8}, 2.0, 2.0));
  EXPECT_NEAR(env.clearance({5, 7}), 1.0, 1e-12);
  EXPECT_NEAR(env.clearance({0.5, 0.5}), 0.5, 1e-12);  // wall
  EXPECT_LT(env.clearance({5, 5}), 0.0);
  EXPECT_TRUE(env.point_free({5, 7}, 0.5));
  EXPECT_FALSE(env.point_free({5, 7}, 1.5));
  EXPECT_FALSE(env.point_free({-1, 5}));
  EXPECT_TRUE(env.segment_free({1, 1}, {9, 1}, 0.4));
  EXPECT_FALSE(env.segment_free({1, 5}, {9, 5}, 0.0));    // through the circle
  EXPECT_FALSE(env.segment_free({0, 8}, {4, 8}, 0.0));    // through the box
  EXPECT_FALSE(env.segment_free({0.1, 1}, {9, 1}, 0.4));  // inflated disc leaves the bounds
  EXPECT_TRUE(env.polygon_free(Polygon::rectangle({8, 2}, 1.0, 1.0)));
  EXPECT_FALSE(env.polygon_free(Polygon::rectangle({5, 5}, 1.0, 1.0)));
  EXPECT_FALSE(env.polygon_free(Polygon::rectangle({9.8, 2}, 1.0, 1.0)));
}

TEST(Environment, RayCastHitsObstaclesAndWalls) {
  Environment2D env({{0, 0}, {10, 10}});
  env.add_obstacle(Circle{{5, 5}, 1.0});
  EXPECT_NEAR(*env.ray_cast({2, 5}, 0.0, 20.0), 2.0, 1e-12);
  EXPECT_NEAR(*env.ray_cast({2, 2}, 0.0, 20.0), 8.0, 1e-12);        // east wall
  EXPECT_FALSE(env.ray_cast({2, 2}, 0.0, 5.0).has_value());         // beyond max range
  EXPECT_NEAR(*env.ray_cast({2, 2}, math::kPi, 20.0), 2.0, 1e-12);  // west wall
}

TEST(EnvironmentProperty, SegmentFreeImpliesSampledPointsFree) {
  Environment2D env({{0, 0}, {20, 20}});
  math::Random rng(testing::kPropertySeed);
  for (int i = 0; i < 15; ++i) {
    env.add_obstacle(Circle{{rng.uniform(2, 18), rng.uniform(2, 18)}, rng.uniform(0.3, 1.5)});
  }
  env.add_obstacle(Polygon::rectangle({10, 10}, 3.0, 1.0, 0.4));
  int free_segments = 0;
  for (int i = 0; i < 2000; ++i) {
    const Vec2 a(rng.uniform(0, 20), rng.uniform(0, 20));
    const Vec2 b(rng.uniform(0, 20), rng.uniform(0, 20));
    if (!env.segment_free(a, b, 0.2)) {
      continue;
    }
    ++free_segments;
    for (int k = 0; k <= 50; ++k) {
      const Vec2 p = a + (b - a) * (k / 50.0);
      ASSERT_GT(env.clearance(p), 0.2 - 1e-9) << "segment " << i << " sample " << k;
    }
  }
  EXPECT_GT(free_segments, 50);
}

TEST(GridSpec, WorldCellRoundTrip) {
  const GridSpec g(10, 5, 0.5, Vec2(-1.0, 2.0));
  EXPECT_EQ(g.world_to_cell({-1.0, 2.0}), (GridIndex{0, 0}));
  EXPECT_EQ(g.world_to_cell({-0.51, 2.49}), (GridIndex{0, 0}));
  EXPECT_EQ(g.world_to_cell({-0.5, 2.5}), (GridIndex{1, 1}));
  EXPECT_EQ(g.world_to_cell({-1.01, 2.0}), (GridIndex{-1, 0}));
  EXPECT_FALSE(g.world_to_cell_checked({-1.01, 2.0}).has_value());
  EXPECT_FALSE(g.world_to_cell_checked({4.0, 2.0}).has_value());  // x = origin + width*res
  const GridIndex c{3, 4};
  EXPECT_EQ(g.world_to_cell(g.cell_center(c)), c);
  EXPECT_EQ(g.from_linear(g.linear_index(c)), c);
  EXPECT_EQ(g.cell_count(), 50U);
  EXPECT_THROW(GridSpec(0, 5, 1.0), std::invalid_argument);
  EXPECT_THROW(GridSpec(5, 5, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace roboticscore::geometry
