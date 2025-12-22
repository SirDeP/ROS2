#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>

class CCDNode : public rclcpp::Node
{
public:
    CCDNode();

private:
    // CCD algorithm parameters
    struct DHParameters {
        double a;      // Link length
        double alpha;  // Link twist
        double d;      // Link offset
        double theta;  // Joint angle (variable)
    };

    // Forward kinematics using DH parameters
    Eigen::Matrix4d dhTransform(double a, double alpha, double d, double theta);
    Eigen::Vector3d forwardKinematics(const std::vector<double>& joint_angles);
    
    // CCD inverse kinematics
    void cyclicCoordinateDescent(double target_x, double target_y, double target_z);
    bool ccdIteration(const Eigen::Vector3d& target, std::vector<double>& joint_angles, int joint_idx);
    
    // Broadcast computed joint states
    void broadcastJointState();
    
    // Parameter callback
    rcl_interfaces::msg::SetParametersResult OnSetParametersCallback(const std::vector<rclcpp::Parameter> &params);

    // Joint angles
    double theta_1_;
    double theta_2_;
    double theta_3_;
    double theta_4_;
    double theta_5_;
    double theta_6_;
    
    // Robot link lengths (from Parol6 DH parameters)
    double L0_;  // a1 - Base height (110.5mm)
    double L1_;  // a2 - Base offset (23.42mm)
    double L2_;  // a3 - Shoulder to elbow (180mm)
    double L3_;  // a4 - Elbow offset (43.5mm)
    double L4_;  // a5 - Elbow to wrist (176.35mm)
    double L5_;  // a6 - Wrist offset (62.8mm)
    double L6_;  // a7 - End effector (45.25mm)
    
    // Target position
    double x_, y_, z_;
    double prev_x_, prev_y_, prev_z_;
    bool target_changed_;
    
    // CCD parameters
    int max_iterations_;
    double tolerance_;
    
    // ROS2 components
    OnSetParametersCallbackHandle::SharedPtr set_param_callback_handle_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};
