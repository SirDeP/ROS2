#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class JointPublisherNode : public rclcpp::Node
{
public:
    JointPublisherNode();

private:
    void inverseKinematics(double x, double z);
    void broadcastJointState();
    rcl_interfaces::msg::SetParametersResult OnSetParametersCallback(const std::vector<rclcpp::Parameter> &params);

    double theta_1_;
    double theta_2_;
    double theta_3_;
    double theta_4_;
    double theta_5_;
    double theta_6_;
    double L1, L2;
    double x_, y_, z_;

    OnSetParametersCallbackHandle::SharedPtr set_param_callback_handle_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};