#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

class ConcatNode : public rclcpp::Node
{
public:
    ConcatNode();

private:

    void topic1_callback(const std_msgs::msg::String::SharedPtr msg);
    void publish_message();

    rclcpp::TimerBase::SharedPtr walltimer_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr topic1_subscriber_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topic1_publisher_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr concatenated_publisher_;

    std::string message_from_topic1_;
};