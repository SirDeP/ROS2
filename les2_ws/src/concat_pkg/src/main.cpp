#include "node/concat.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConcatNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}