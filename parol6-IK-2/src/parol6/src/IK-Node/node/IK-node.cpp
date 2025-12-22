#include "IK-node.hpp"
#include <algorithm>  // for std::clamp
#include <cmath>      // for sqrt, atan2

JointPublisherNode::JointPublisherNode() : Node("ik_node")
{
    // Robot parameters from URDF
    this->declare_parameter<double>("L1", 0.18);     // shoulder to elbow (L2 joint -Y offset)
    this->declare_parameter<double>("L2", 0.21985);  // elbow to wrist (0.0435 + 0.17635)
    this->declare_parameter<double>("x", 0.2);
    this->declare_parameter<double>("y", 0.00);
    this->declare_parameter<double>("z", 0.45);
    x_ = this->get_parameter("x").as_double();
    y_ = this->get_parameter("y").as_double();
    z_ = this->get_parameter("z").as_double();
    // Declare and get parameters
    this->declare_parameter<double>("theta_1", 0.0);
    this->declare_parameter<double>("theta_2", 0.0);
    this->declare_parameter<double>("theta_3", 0.0);
    this->declare_parameter<double>("theta_4", 0.0);
    this->declare_parameter<double>("theta_5", 0.0);
    this->declare_parameter<double>("theta_6", 0.0);
    theta_1_ = this->get_parameter("theta_1").as_double();
    theta_2_ = this->get_parameter("theta_2").as_double();
    theta_3_ = this->get_parameter("theta_3").as_double();
    theta_4_ = this->get_parameter("theta_4").as_double();
    theta_5_ = this->get_parameter("theta_5").as_double();
    theta_6_ = this->get_parameter("theta_6").as_double();

    L1 = this->get_parameter("L1").as_double();
    L2 = this->get_parameter("L2").as_double();

    set_param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &JointPublisherNode::OnSetParametersCallback,
        this,
        std::placeholders::_1));
    // Initialize the joint state publisher
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    // Initialize the transform buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Set up a timer to periodically check for the transform and broadcast it
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JointPublisherNode::broadcastJointState, this));
}

rcl_interfaces::msg::SetParametersResult JointPublisherNode::OnSetParametersCallback(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : params) {
        if (param.get_name() == "x") {
            x_ = param.as_double();
        } else if (param.get_name() == "y") {
            y_ = param.as_double();
        } else if (param.get_name() == "z") {
            z_ = param.as_double();
        } else if (param.get_name() == "theta_1") {
            theta_1_ = param.as_double();
        } else if (param.get_name() == "theta_2") {
            theta_2_ = param.as_double();
        } else if (param.get_name() == "theta_3") {
            theta_3_ = param.as_double();
        } else if (param.get_name() == "theta_4") {
            theta_4_ = param.as_double();
        } else if (param.get_name() == "theta_5") {
            theta_5_ = param.as_double();
        } else if (param.get_name() == "theta_6") {
            theta_6_ = param.as_double();
        }
    }
    return result;
}

void JointPublisherNode::inverseKinematics(double /*x*/, double /*z*/)
{
    // ================================================================
    // PAROL6 ROBOT INVERSE KINEMATICS
    // ================================================================
    //
    // URDF: L2 joint has rpy="-π/2 0 0" which rotates the frame
    // When L2=0: arm points along +Y in rotated frame = +Z in world frame (UP)
    // Positive L2: rotates around Z in rotated frame, which is -Y in world
    //              This brings the arm FORWARD and DOWN
    //
    // L3 joint: axis="0 0 -1" means positive angle closes the elbow
    // ================================================================

    const double shoulder_height = 0.1105;
    const double shoulder_offset_x = 0.0234;
    const double gripper_length = 0.0628;  // Distance from wrist to gripper tip

    // ================================================================
    // Joint 1: Base rotation
    // ================================================================
    theta_1_ = atan2(y_, x_);

    // ================================================================
    // 2D IK in the vertical plane
    // ================================================================
    double r = sqrt(x_ * x_ + y_ * y_);  // Horizontal distance from base origin
    // double z_rel = z_ - shoulder_height;  // Height relative to shoulder
    
    // double d_sq = r * r + z_rel * z_rel;
    // double d = sqrt(d_sq);

    // // Elbow angle using law of cosines
    // double cos_elbow = (d_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    // if (cos_elbow < -1.0 || cos_elbow > 1.0) {
    //     RCLCPP_WARN(this->get_logger(), 
    //         "Target (%.3f, %.3f, %.3f) unreachable! cos=%.4f, d=%.3f, max=%.3f",
    //         x_, y_, z_, cos_elbow, d, L1 + L2);
    //     cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);
    // }

    // // Elbow DOWN configuration: positive sine
    // double sin_elbow = sqrt(1.0 - cos_elbow * cos_elbow);
    // double elbow = atan2(sin_elbow, cos_elbow);

    // // Shoulder angle (measured from HORIZONTAL forward, standard IK convention)
    // double k1 = L1 + L2 * cos_elbow;
    // double k2 = L2 * sin_elbow;
    // double beta = atan2(k2, k1);
    // double alpha = atan2(z_rel, r);  // Angle from horizontal
    // double shoulder = alpha - beta;   // Angle from horizontal (+ve = up, -ve = down)

    // ================================================================
    // Map to URDF joints with proper frame compensation
    // ================================================================
    // L2: Negate shoulder angle - L2=0 means horizontal, L2>0 lifts up
    // theta_2_ = -shoulder;

    // L3: Try without negation
    // theta_3_ = elbow - 0.5*M_PI;

    // Joints 4-6: User will set these


    // Adjust for shoulder position  
    double targetX = r - shoulder_offset_x;  // radial distance from shoulder to target
    double targetY = z_ + gripper_length - shoulder_height;  // wrist height (gripper tip + gripper length)

    // cos(θ₂) = (x² + y² - L₁² - L₂²) / (2·L₁·L₂)
    double d_sq = targetX * targetX + targetY * targetY;
    double cos_theta2 = (d_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    // Check reachability
    if (cos_theta2 < -1.0 || cos_theta2 > 1.0)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Target (%.3f, %.3f, %.3f) unreachable! cos=%.4f",
                    x_, y_, z_, cos_theta2);
        cos_theta2 = std::clamp(cos_theta2, -1.0, 1.0);
    }

    // ELBOW UP: sin(θ₂) is NEGATIVE
    double sin_theta2 = -sqrt(1.0 - cos_theta2 * cos_theta2);
    double theta2 = atan2(sin_theta2, cos_theta2);

    double k1 = L1 + L2 * cos_theta2;
    double k2 = L2 * sin_theta2;
    double gamma = atan2(k2, k1);
    double theta1 = atan2(targetY, targetX) - gamma;

    theta_2_ = -theta1 + M_PI / 2.0;  // Shoulder (negated to increase forward)
    theta_3_ = -theta2;               // Elbow (negative because axis is -Z)
    
    // Wrist pitch: compensate for shoulder and elbow to keep gripper pointing down
    double theta3 = theta1 + theta2 + M_PI / 2.0;  // Adjusted for negated theta_2_
    theta_4_ = -theta3;               // Wrist pitch (negative because axis is -Z)
    theta_5_ = 0.0;                  // Wrist roll
    theta_6_ = 0.0;                  // Gripper rotation

    // y = -y;
    RCLCPP_INFO(this->get_logger(), "IK: target=(%.3f,%.3f,%.3f) -> L1=%.3f° L2=%.3f° L3=%.3f°",
                x_, y_, z_, theta_1_*180/M_PI, theta_2_*180/M_PI, theta_3_*180/M_PI);
}

void JointPublisherNode::broadcastJointState()
{
    inverseKinematics(x_, z_);
    
    // Create a JointState message
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name = {"L1", "L2", "L3", "L4", "L5", "L6"};
    joint_state.position = {theta_1_, theta_2_, theta_3_, theta_4_, theta_5_, theta_6_};
    
    RCLCPP_INFO(this->get_logger(), "Joint states (%.3f %.3f %.3f %.3f %.3f %.3f) published",
                theta_1_, theta_2_, theta_3_, theta_4_, theta_5_, theta_6_);
    
    // Publish the joint state
    joint_state_publisher_->publish(joint_state);
}
