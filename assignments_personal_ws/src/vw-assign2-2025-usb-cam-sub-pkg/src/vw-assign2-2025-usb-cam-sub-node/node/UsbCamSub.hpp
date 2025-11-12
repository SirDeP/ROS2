#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace assignment::two::usb_cam_sub
{

class UsbCamSub : public rclcpp::Node
{
public:
    UsbCamSub();
    // ~UsbCamSub();

private:
    int edge_detection_choice_;
    int circle_radius_;
    int circle_thickness_;
    std::vector<uint8_t> circle_color_;

    cv::Mat cv_frame_;
    cv::Mat transformed_frame_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_camera_tl_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_camera_tr_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_camera_bl_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_camera_br_;

    void subscriber_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void image_processing();
};

}  // namespace assignment::two::usb_cam_sub
