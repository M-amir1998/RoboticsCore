#include <memory>

#include "roboticscore_ros/grid_planner_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roboticscore_ros::GridPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
