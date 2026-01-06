#include "opencv.hpp"

namespace opencv_test
{

TestNode::TestNode(std::unique_ptr<DatabaseManager> db_manager)
: Node("opencv_test_node")
{
    RCLCPP_INFO(this->get_logger(), "OpenCV Test Node has been started.");
    // Additional initialization code can be added here
}

}  // namespace opencv_test