#include "nodes/opencv.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<opencv_test::IMUDatabaseWriter>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
