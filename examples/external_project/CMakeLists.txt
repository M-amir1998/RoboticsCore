# Minimal downstream project consuming an installed RoboticsCore:
#
#   cmake --install build/release --prefix <prefix>
#   cmake -S examples/external_project -B build/external -DCMAKE_PREFIX_PATH=<prefix>
#   cmake --build build/external && ./build/external/plan_and_track
cmake_minimum_required(VERSION 3.25)
project(roboticscore_external_example LANGUAGES CXX)

find_package(RoboticsCore 0.1 CONFIG REQUIRED)

add_executable(plan_and_track main.cpp)
target_compile_features(plan_and_track PRIVATE cxx_std_20)
# Link only the modules that are needed, or RoboticsCore::RoboticsCore for everything.
target_link_libraries(plan_and_track PRIVATE RoboticsCore::planning RoboticsCore::control
                                             RoboticsCore::trajectory)
