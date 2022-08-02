#include "vl53l5cx/vl53l5cx_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<vl53l5cx::VL53L5CXNode>("vl53l5cx");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
