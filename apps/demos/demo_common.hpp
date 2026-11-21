#pragma once

// Shared helpers for the demo executable: JSON serialisation of library types and the
// demo registry. Traces are compact (4 decimals) because they feed the renderers only.

#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "roboticscore/core/types.hpp"
#include "roboticscore/geometry/collision.hpp"
#include "roboticscore/geometry/environment.hpp"
#include "roboticscore/geometry/grid.hpp"
#include "roboticscore/geometry/se2.hpp"

namespace roboticscore::demos {

using json = nlohmann::json;

inline double r4(double v) {
  return std::round(v * 1e4) / 1e4;
}
inline json to_json(const Vec2& p) {
  return json::array({r4(p.x()), r4(p.y())});
}
inline json to_json(const geometry::Pose2& p) {
  return json::array({r4(p.x()), r4(p.y()), r4(p.theta())});
}
inline json to_json(const std::vector<Vec2>& pts) {
  json a = json::array();
  for (const auto& p : pts)
    a.push_back(to_json(p));
  return a;
}
inline json to_json(const std::vector<geometry::Pose2>& poses) {
  json a = json::array();
  for (const auto& p : poses)
    a.push_back(to_json(p));
  return a;
}
inline json cov2(const Mat2& c) {
  return json::array({r4(c(0, 0)), r4(c(0, 1)), r4(c(1, 1))});
}

inline json to_json(const geometry::Environment2D& env) {
  json j;
  j["bounds"] = json::array({r4(env.bounds().min.x()), r4(env.bounds().min.y()),
                             r4(env.bounds().max.x()), r4(env.bounds().max.y())});
  j["circles"] = json::array();
  for (const auto& c : env.circles()) {
    j["circles"].push_back(json::array({r4(c.center.x()), r4(c.center.y()), r4(c.radius)}));
  }
  j["polygons"] = json::array();
  for (const auto& p : env.polygons())
    j["polygons"].push_back(to_json(p.vertices()));
  return j;
}

inline json to_json(const geometry::GridSpec& g) {
  return {{"width", g.width()},
          {"height", g.height()},
          {"resolution", g.resolution()},
          {"origin", to_json(g.origin())}};
}

/// Demo parameters: defaults overridden by an optional JSON config file.
struct DemoContext {
  std::uint64_t seed = 1;
  json config = json::object();

  template <typename T>
  T get(const std::string& key, T fallback) const {
    return config.contains(key) ? config.at(key).get<T>() : fallback;
  }
};

using DemoFn = std::function<json(const DemoContext&)>;

/// Registry of all demos (populated by register_*_demos()).
std::map<std::string, std::pair<std::string, DemoFn>>& registry();

void register_planning_demos();
void register_estimation_demos();
void register_control_demos();
void register_integrated_demos();

}  // namespace roboticscore::demos
