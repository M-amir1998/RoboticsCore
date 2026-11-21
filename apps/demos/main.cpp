// roboticscore_demo: runs a named scenario and writes a JSON trace for the renderers.
//
//   roboticscore_demo --list
//   roboticscore_demo astar --seed 3 --config configs/planning/astar.json --out astar.json

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>

#include "roboticscore/core/version.hpp"

#include "demo_common.hpp"

namespace roboticscore::demos {

std::map<std::string, std::pair<std::string, DemoFn>>& registry() {
  static std::map<std::string, std::pair<std::string, DemoFn>> demos;
  return demos;
}

}  // namespace roboticscore::demos

namespace {

int usage() {
  std::cerr << "usage: roboticscore_demo <demo> [--seed N] [--config file.json] [--out file]\n"
               "       roboticscore_demo --list\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace roboticscore::demos;
  register_planning_demos();
  register_estimation_demos();
  register_control_demos();
  register_integrated_demos();
  if (argc < 2) return usage();
  const std::string name = argv[1];
  if (name == "--list") {
    for (const auto& [key, value] : registry()) {
      std::cout << key << "  -  " << value.first << "\n";
    }
    return 0;
  }
  const auto it = registry().find(name);
  if (it == registry().end()) {
    std::cerr << "unknown demo '" << name << "' (use --list)\n";
    return 2;
  }
  DemoContext ctx;
  std::string out_path = name + ".trace.json";
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--seed" && i + 1 < argc) {
      ctx.seed = std::strtoull(argv[++i], nullptr, 10);
    } else if (arg == "--config" && i + 1 < argc) {
      std::ifstream in(argv[++i]);
      if (!in) {
        std::cerr << "cannot open config " << argv[i] << "\n";
        return 2;
      }
      ctx.config = json::parse(in);
      if (ctx.config.contains("seed")) ctx.seed = ctx.config["seed"].get<std::uint64_t>();
    } else if (arg == "--out" && i + 1 < argc) {
      out_path = argv[++i];
    } else {
      return usage();
    }
  }
  try {
    json trace = it->second.second(ctx);
    trace["demo"] = name;
    trace["seed"] = ctx.seed;
    trace["config"] = ctx.config;
    trace["library_version"] = std::string(roboticscore::version());
    std::ofstream out(out_path);
    out << trace.dump();
    std::cout << "wrote " << out_path << "\n";
  } catch (const std::exception& e) {
    std::cerr << "demo '" << name << "' failed: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
