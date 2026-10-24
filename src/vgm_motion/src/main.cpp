#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "vgm_motion/motion_server.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("motion_server");

  try {
    auto server = std::make_unique<vgm_motion::MotionServer>(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    server.reset();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "Motion server failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
