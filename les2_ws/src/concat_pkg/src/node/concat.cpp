#include "concat.hpp"

ConcatNode::ConcatNode() : Node("concat_node")
{
    topic1_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "vin10", 10,
        std::bind(&ConcatNode::topic1_callback, this, std::placeholders::_1));
    
    topic1_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "vin1", 10);

    concatenated_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "jan1", 10);

    walltimer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&ConcatNode::publish_message, this));
}

void ConcatNode::topic1_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string message_from_topic1_ = msg->data;
    std::string concatenated_message = message_from_topic1_;

    auto output_msg = std_msgs::msg::String();
    output_msg.data = concatenated_message;

    concatenated_publisher_->publish(output_msg);
}
    
void ConcatNode::publish_message()
{
    auto message = std_msgs::msg::String();
    message.data = "Ik ";

    topic1_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published message to topic1: '%s'", message.data.c_str());
}