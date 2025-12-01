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
    //Params
    int edge_detection_choice_;
    int circle_radius_;
    int circle_thickness_;
    std::vector<uint8_t> circle_color_;
    int canny_threshold1_;
    int canny_threshold2_;

    //OpenCV Mats
    cv::Mat cv_frame_;
    cv::Mat transformed_frame_;

    //ROS2
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_camera_;

    //Functions
    rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter> & params);
    void subscriber_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void image_processing();
};

// compile-time FNV-1a hash for string literals
constexpr uint64_t constexpr_hash(const char *s, uint64_t h = 1469598103934665603ULL) {
    return (*s) ? constexpr_hash(s + 1, (h ^ static_cast<uint64_t>(*s)) * 1099511628211ULL) : h;
}

// runtime FNV-1a hash for std::string
inline uint64_t runtime_hash(const std::string &s) {
    uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : s) {
        h = (h ^ static_cast<uint64_t>(c)) * 1099511628211ULL;
    }
    return h;
}


}  // namespace assignment::two::usb_cam_sub
