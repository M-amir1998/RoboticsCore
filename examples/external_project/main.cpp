// Plans a grid path with A*, smooths it into a reference path and tracks it with Pure Pursuit.
#include <cstdio>
#include <string>
#include <vector>

#include "roboticscore/control/geometric_trackers.hpp"
#include "roboticscore/core/version.hpp"
#include "roboticscore/planning/grid/grid_search.hpp"
#include "roboticscore/trajectory/reference_path.hpp"

int main() {
  using namespace roboticscore;
  planning::GridMap map(geometry::GridSpec(40, 30, 0.5));
  for (int y = 0; y < 22; ++y)
    map.set_blocked({20, y}, true);  // a wall with a gap at the top

  const planning::GridPath plan = planning::astar(map, {2, 2}, {37, 3});
  if (!plan.found()) {
    std::puts("no path");
    return 1;
  }
  auto waypoints = planning::to_world(map.spec(), plan.cells);
  const auto path = trajectory::ReferencePath::from_waypoints(waypoints, 0.25, 2.0);

  const models::BicycleParams car{1.0, 0.6, 3.0, 2.0};
  control::PurePursuitTracker tracker(car);
  const models::KinematicBicycle model(car);
  models::BicycleState x(waypoints.front().x(), waypoints.front().y(), 1.57, 0.0);
  int steps = 0;
  while (!tracker.status().finished && steps < 3000) {
    const auto u = tracker.compute(x, path, 0.05);
    x = model.step(x, models::BicycleInput(u.acceleration, u.steering), 0.05);
    ++steps;
  }
  std::printf("RoboticsCore %s: path %.2f m, tracked in %d steps, final cross-track %.3f m\n",
              std::string(version()).c_str(), plan.cost, steps, tracker.status().cross_track_error);
  return tracker.status().finished ? 0 : 1;
}
