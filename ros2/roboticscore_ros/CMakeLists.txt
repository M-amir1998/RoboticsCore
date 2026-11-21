cmake_minimum_required(VERSION 3.22)
project(roboticscore_ros VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# The core library, installed separately (see README.md); point CMAKE_PREFIX_PATH at it.
find_package(RoboticsCore 0.1 CONFIG REQUIRED)

# ROS-independent conversion logic.
add_library(roboticscore_ros_conversions STATIC src/conversions.cpp)
target_include_directories(roboticscore_ros_conversions
                           PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(roboticscore_ros_conversions PUBLIC RoboticsCore::planning)
set_target_properties(roboticscore_ros_conversions PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Node implementations, shared by the executables and the tests.
add_library(roboticscore_ros_nodes STATIC src/grid_planner_node.cpp src/path_tracker_node.cpp)
target_link_libraries(roboticscore_ros_nodes PUBLIC roboticscore_ros_conversions
                                                    RoboticsCore::planning RoboticsCore::control
                                                    RoboticsCore::trajectory)
set_target_properties(roboticscore_ros_nodes PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(roboticscore_ros_nodes PUBLIC rclcpp builtin_interfaces geometry_msgs
                            nav_msgs)
endif()

add_executable(grid_planner src/grid_planner_main.cpp)
target_link_libraries(grid_planner PRIVATE roboticscore_ros_nodes)
add_executable(path_tracker src/path_tracker_main.cpp)
target_link_libraries(path_tracker PRIVATE roboticscore_ros_nodes)

install(TARGETS grid_planner path_tracker DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions roboticscore_ros_conversions)
  ament_add_gtest(test_nodes test/test_nodes.cpp TIMEOUT 120)
  target_link_libraries(test_nodes roboticscore_ros_nodes)
endif()

ament_package()
