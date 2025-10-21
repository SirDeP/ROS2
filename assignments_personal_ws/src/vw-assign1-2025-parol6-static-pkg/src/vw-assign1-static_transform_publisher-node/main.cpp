#include "nodes/static_transform_publisher.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTfNode>());
  rclcpp::shutdown();
  return 0;
}
