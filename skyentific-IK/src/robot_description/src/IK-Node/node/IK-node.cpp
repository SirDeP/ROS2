#include "IK-node.hpp"

JointPublisherNode::JointPublisherNode() : Node("ik_node")
{
    // Robot parameters from URDF
    this->declare_parameter<double>("L1", 0.225);             // joint_2 to joint_3
    this->declare_parameter<double>("L2", 0.200);             // joint_3 to joint_4 (URDF: x=0.200)
    this->declare_parameter<double>("gripper_offset", 0.193); // joint_5 to gripper center (0.0915 + 0.1015)
    this->declare_parameter<double>("x", 0.2);
    this->declare_parameter<double>("y", 0.00);
    this->declare_parameter<double>("z", 0.3);
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
    gripper_offset_ = this->get_parameter("gripper_offset").as_double();

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

rcl_interfaces::msg::SetParametersResult JointPublisherNode::OnSetParametersCallback(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : params)
    {
        if (param.get_name() == "x")
        {
            x_ = param.as_double();
        }
        else if (param.get_name() == "y")
        {
            y_ = param.as_double();
        }
        else if (param.get_name() == "z")
        {
            z_ = param.as_double();
        }
    }
    return result;
}

void JointPublisherNode::inverseKinematics(double /*x*/, double /*z*/)
{
    const double shoulder_height = 0.0965 + 0.150; // = 0.2465m

    theta_1_ = atan2(y_, x_);

    double adjusted_z = z_ + gripper_offset_;

    // Convert 3D target to 2D IK plane
    double targetX = sqrt(x_ * x_ + y_ * y_);      // radial distance
    double targetY = adjusted_z - shoulder_height; // height above shoulder

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

    double theta3 = -theta1 - theta2 - M_PI / 2.0;

    theta_2_ = theta1 - M_PI / 2.0;   // Shoulder (compensate for URDF frame)
    theta_3_ = theta2;                // Elbow
    theta_4_ = theta3;                // Wrist pitch
    theta_5_ = theta_1_ - M_PI / 2.0; // Wrist roll
    theta_6_ = 0.0;                   // Gripper
}

void JointPublisherNode::broadcastJointState()
{
    inverseKinematics(x_, z_);
    // Create a JointState message
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    joint_state.position = {theta_1_, theta_2_, theta_3_, theta_4_, theta_5_, theta_6_};
    RCLCPP_INFO(this->get_logger(), "Joint states (%f %f %f %f %f %f) were sent to the system.",
                theta_1_, theta_2_, theta_3_, theta_4_,
                theta_5_, theta_6_);
    // Publish the joint state
    joint_state_publisher_->publish(joint_state);
}
