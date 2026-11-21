#include <chrono>

#include "pipelines.hpp"

namespace roboticscore::pipelines {

TrackingRun run_tracking(control::PathTracker& tracker, const trajectory::ReferencePath& path,
                         const models::BicycleParams& vehicle, const models::BicycleState& start,
                         double dt, int max_steps) {
  TrackingRun run;
  const models::KinematicBicycle car(vehicle);
  models::BicycleState x = start;
  tracker.reset();
  for (int k = 0; k < max_steps; ++k) {
    const auto t0 = std::chrono::steady_clock::now();
    const control::VehicleCommand u = tracker.compute(x, path, dt);
    const auto t1 = std::chrono::steady_clock::now();
    run.compute_time_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    run.cross_track.push_back(tracker.status().cross_track_error);
    run.heading_error.push_back(tracker.status().heading_error);
    run.steering.push_back(u.steering);
    run.acceleration.push_back(u.acceleration);
    x = car.step(x, models::BicycleInput(u.acceleration, u.steering), dt);
    run.states.push_back(x);
    if (tracker.status().finished) {
      run.finished = true;
      break;
    }
  }
  return run;
}

}  // namespace roboticscore::pipelines
