#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class TestNode : public rclcpp::Node {
public:
    TestNode(std::unique_ptr<DatabaseManager> db_manager = nullptr);

private:

};
