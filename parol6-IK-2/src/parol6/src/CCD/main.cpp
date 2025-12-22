#include "node/CCD-node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CCDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
