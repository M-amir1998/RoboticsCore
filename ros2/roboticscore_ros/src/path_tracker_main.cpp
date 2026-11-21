#include <memory>

#include "roboticscore_ros/path_tracker_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roboticscore_ros::PathTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
