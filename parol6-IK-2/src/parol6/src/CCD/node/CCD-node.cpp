#include "CCD-node.hpp"
#include <algorithm>
#include <cmath>

CCDNode::CCDNode() : Node("ccd_node")
{
    // Robot parameters from Parol6 kinematic structure
    // a1=110.5mm, a2=23.42mm, a3=180mm, a4=43.5mm, a5=176.35mm, a6=62.8mm, a7=45.25mm
    this->declare_parameter<double>("L0", 0.11050);  // a1 - Base height
    this->declare_parameter<double>("L1", 0.02342);  // a2 - Base offset
    this->declare_parameter<double>("L2", 0.18000);  // a3 - Shoulder to elbow
    this->declare_parameter<double>("L3", 0.04350);  // a4 - Elbow offset
    this->declare_parameter<double>("L4", 0.17635);  // a5 - Elbow to wrist
    this->declare_parameter<double>("L5", 0.06280);  // a6 - Wrist offset
    this->declare_parameter<double>("L6", 0.04525);  // a7 - End effector
    
    // Target position parameters
    this->declare_parameter<double>("x", 0.2);
    this->declare_parameter<double>("y", 0.00);
    this->declare_parameter<double>("z", 0.45);
    
    // Joint angle parameters
    this->declare_parameter<double>("theta_1", 0.0);
    this->declare_parameter<double>("theta_2", 0.0);
    this->declare_parameter<double>("theta_3", 0.0);
    this->declare_parameter<double>("theta_4", 0.0);
    this->declare_parameter<double>("theta_5", 0.0);
    this->declare_parameter<double>("theta_6", 0.0);
    
    // CCD algorithm parameters
    this->declare_parameter<int>("max_iterations", 100);
    this->declare_parameter<double>("tolerance", 0.001);
    
    // Get parameters
    L0_ = this->get_parameter("L0").as_double();  // a1 - Base height
    L1_ = this->get_parameter("L1").as_double();  // a2 - Base offset
    L2_ = this->get_parameter("L2").as_double();  // a3 - Shoulder to elbow
    L3_ = this->get_parameter("L3").as_double();  // a4 - Elbow offset
    L4_ = this->get_parameter("L4").as_double();  // a5 - Elbow to wrist
    L5_ = this->get_parameter("L5").as_double();  // a6 - Wrist offset  
    L6_ = this->get_parameter("L6").as_double();  // a7 - End effector
    
    x_ = this->get_parameter("x").as_double();
    y_ = this->get_parameter("y").as_double();
    z_ = this->get_parameter("z").as_double();
    
    // Initialize previous target
    prev_x_ = x_;
    prev_y_ = y_;
    prev_z_ = z_;
    target_changed_ = true;
    
    theta_1_ = this->get_parameter("theta_1").as_double();
    theta_2_ = this->get_parameter("theta_2").as_double();
    theta_3_ = this->get_parameter("theta_3").as_double();
    theta_4_ = this->get_parameter("theta_4").as_double();
    theta_5_ = this->get_parameter("theta_5").as_double();
    theta_6_ = this->get_parameter("theta_6").as_double();
    
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    tolerance_ = this->get_parameter("tolerance").as_double();
    
    // Set up parameter callback
    set_param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &CCDNode::OnSetParametersCallback,
        this,
        std::placeholders::_1));
    
    // Initialize the joint state publisher
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    
    // Initialize the transform buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Set up a timer to periodically compute IK and broadcast joint states
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CCDNode::broadcastJointState, this));
    
    RCLCPP_INFO(this->get_logger(), "CCD Node initialized with a1=%.3f, a2=%.3f, a3=%.3f, a4=%.3f, a5=%.3f, a6=%.3f, a7=%.3f",
                L0_, L1_, L2_, L3_, L4_, L5_, L6_);
    
    // Test FK at standby position: URDF joints should be [0, -π/2, π, 0, 0, π]
    std::vector<double> standby = {0.0, -M_PI/2.0, M_PI, 0.0, 0.0, M_PI};
    Eigen::Vector3d standby_pos = forwardKinematics(standby);
    RCLCPP_INFO(this->get_logger(), "FK Test - Standby position: (%.3f, %.3f, %.3f)",
                standby_pos.x(), standby_pos.y(), standby_pos.z());
    
    // Test FK at zero position
    std::vector<double> zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d zero_pos = forwardKinematics(zero);
    RCLCPP_INFO(this->get_logger(), "FK Test - Zero position: (%.3f, %.3f, %.3f)",
                zero_pos.x(), zero_pos.y(), zero_pos.z());
}

rcl_interfaces::msg::SetParametersResult CCDNode::OnSetParametersCallback(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto & param : params) {
        if (param.get_name() == "x") {
            if (std::abs(x_ - param.as_double()) > 1e-6) {
                x_ = param.as_double();
                target_changed_ = true;
            }
        } else if (param.get_name() == "y") {
            if (std::abs(y_ - param.as_double()) > 1e-6) {
                y_ = param.as_double();
                target_changed_ = true;
            }
        } else if (param.get_name() == "z") {
            if (std::abs(z_ - param.as_double()) > 1e-6) {
                z_ = param.as_double();
                target_changed_ = true;
            }
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
        } else if (param.get_name() == "max_iterations") {
            max_iterations_ = param.as_int();
        } else if (param.get_name() == "tolerance") {
            tolerance_ = param.as_double();
        }
    }
    
    return result;
}

Eigen::Matrix4d CCDNode::dhTransform(double a, double alpha, double d, double theta)
{
    // Denavit-Hartenberg transformation matrix
    Eigen::Matrix4d T;
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    
    T << ct,    -st*ca,   st*sa,   a*ct,
         st,     ct*ca,  -ct*sa,   a*st,
         0,      sa,      ca,      d,
         0,      0,       0,       1;
    
    return T;
}

Eigen::Vector3d CCDNode::forwardKinematics(const std::vector<double>& joint_angles)
{
    // Forward kinematics by chaining URDF transformations directly
    // Using proper RPY to rotation matrix conversion: R = R_z(yaw) * R_y(pitch) * R_x(roll)
    
    double q1 = joint_angles[0];  // L1 - Base rotation around Z
    double q2 = joint_angles[1];  // L2 - Shoulder 
    double q3 = joint_angles[2];  // L3 - Elbow 
    double q4 = joint_angles[3];  // L4 - Wrist pitch
    double q5 = joint_angles[4];  // L5 - Wrist roll
    double q6 = joint_angles[5];  // L6 - Gripper rotation
    
    // Helper lambda for RPY to rotation matrix
    auto rpy_to_matrix = [](double roll, double pitch, double yaw) {
        Eigen::Matrix3d R;
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw), sy = sin(yaw);
        
        // R = R_z(yaw) * R_y(pitch) * R_x(roll)
        R(0,0) = cy*cp;
        R(0,1) = cy*sp*sr - sy*cr;
        R(0,2) = cy*sp*cr + sy*sr;
        R(1,0) = sy*cp;
        R(1,1) = sy*sp*sr + cy*cr;
        R(1,2) = sy*sp*cr - cy*sr;
        R(2,0) = -sp;
        R(2,1) = cp*sr;
        R(2,2) = cp*cr;
        
        return R;
    };
    
    // Start with identity
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // L1 joint: rotation around Z axis by q1
    Eigen::Matrix4d T_L1 = Eigen::Matrix4d::Identity();
    T_L1.block<3,3>(0,0) = Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T = T * T_L1;
    
    // L1 to L2: origin xyz="0.0234207 0 0.1105" rpy="-1.5707963267949 0 0"
    Eigen::Matrix4d T_L1_to_L2 = Eigen::Matrix4d::Identity();
    T_L1_to_L2.block<3,1>(0,3) << 0.0234207, 0, 0.1105;
    T_L1_to_L2.block<3,3>(0,0) = rpy_to_matrix(-M_PI/2, 0, 0);
    T = T * T_L1_to_L2;
    
    // L2 joint: rotation around Z axis by q2 (in L2 frame)
    Eigen::Matrix4d T_L2 = Eigen::Matrix4d::Identity();
    T_L2.block<3,3>(0,0) = Eigen::AngleAxisd(q2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T = T * T_L2;
    
    // L2 to L3: origin xyz="0 -0.18 0" rpy="3.1416 0 -1.5708"
    Eigen::Matrix4d T_L2_to_L3 = Eigen::Matrix4d::Identity();
    T_L2_to_L3.block<3,1>(0,3) << 0, -0.18, 0;
    T_L2_to_L3.block<3,3>(0,0) = rpy_to_matrix(M_PI, 0, -M_PI/2);
    T = T * T_L2_to_L3;
    
    // L3 joint: rotation around -Z axis by q3 (axis="0 0 -1")
    Eigen::Matrix4d T_L3 = Eigen::Matrix4d::Identity();
    T_L3.block<3,3>(0,0) = Eigen::AngleAxisd(-q3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T = T * T_L3;
    
    // L3 to L4: origin xyz="0.0435 0 0" rpy="1.5707963267949 0 3.14159265358979"
    Eigen::Matrix4d T_L3_to_L4 = Eigen::Matrix4d::Identity();
    T_L3_to_L4.block<3,1>(0,3) << 0.0435, 0, 0;
    T_L3_to_L4.block<3,3>(0,0) = rpy_to_matrix(M_PI/2, 0, M_PI);
    T = T * T_L3_to_L4;
    
    // L4 joint: rotation around -Z axis by q4 (axis="0 0 -1")
    Eigen::Matrix4d T_L4 = Eigen::Matrix4d::Identity();
    T_L4.block<3,3>(0,0) = Eigen::AngleAxisd(-q4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T = T * T_L4;
    
    // L4 to L5: origin xyz="0 0 -0.17635" rpy="-1.5708 0 0"
    Eigen::Matrix4d T_L4_to_L5 = Eigen::Matrix4d::Identity();
    T_L4_to_L5.block<3,1>(0,3) << 0, 0, -0.17635;
    T_L4_to_L5.block<3,3>(0,0) = rpy_to_matrix(-M_PI/2, 0, 0);
    T = T * T_L4_to_L5;
    
    // L5 joint: rotation around -Z axis by q5 (axis="0 0 -1")
    Eigen::Matrix4d T_L5 = Eigen::Matrix4d::Identity();
    T_L5.block<3,3>(0,0) = Eigen::AngleAxisd(-q5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T = T * T_L5;
    
    // L5 to L6: origin xyz="0 0 0" rpy="1.5708 0 0"
    Eigen::Matrix4d T_L5_to_L6 = Eigen::Matrix4d::Identity();
    T_L5_to_L6.block<3,3>(0,0) = rpy_to_matrix(M_PI/2, 0, 0);
    T = T * T_L5_to_L6;
    
    // L6 joint: rotation around -Z axis by q6 (axis="0 0 -1")
    Eigen::Matrix4d T_L6 = Eigen::Matrix4d::Identity();
    T_L6.block<3,3>(0,0) = Eigen::AngleAxisd(-q6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T = T * T_L6;
    
    // The end effector is at the origin of L6 frame (gripper attachment point)
    Eigen::Vector4d end_effector_homogeneous(0, 0, 0, 1);
    end_effector_homogeneous = T * end_effector_homogeneous;
    
    // Debug: print Z value breakdown
    Eigen::Vector3d pos = end_effector_homogeneous.head<3>();
    // RCLCPP_INFO(this->get_logger(), "FK Z breakdown: q1=%.3f q2=%.3f q3=%.3f q4=%.3f -> (%.3f, %.3f, %.3f)", 
    //             q1, q2, q3, q4, pos.x(), pos.y(), pos.z());
    
    return pos;
}

bool CCDNode::ccdIteration(const Eigen::Vector3d& target, std::vector<double>& joint_angles, int joint_idx)
{
    // Get current end-effector position
    Eigen::Vector3d end_effector = forwardKinematics(joint_angles);
    
    // Check if already at target
    double current_error = (target - end_effector).norm();
    if (current_error < tolerance_) {
        return true;
    }
    
    // Compute position of the joint we're adjusting
    Eigen::Vector3d joint_pos;
    
    double q1 = joint_angles[0];
    double q2 = joint_angles[1];
    double q3 = joint_angles[2];
    
    if (joint_idx == 0) {
        // Base joint is at origin
        joint_pos = Eigen::Vector3d(0, 0, 0);
    } else {
        // Build FK up to the joint position
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        // Always include base rotation
        Eigen::Matrix4d T_L1 = Eigen::Matrix4d::Identity();
        T_L1(0, 0) = cos(q1);
        T_L1(0, 1) = -sin(q1);
        T_L1(1, 0) = sin(q1);
        T_L1(1, 1) = cos(q1);
        T = T * T_L1;
        
        if (joint_idx == 1) {
            // Joint 1 position (at L2 joint origin)
            Eigen::Matrix4d T_L1_to_L2 = Eigen::Matrix4d::Identity();
            T_L1_to_L2(0, 3) = 0.0234207;
            T_L1_to_L2(2, 3) = 0.1105;
            T = T * T_L1_to_L2;
        } else {
            // Include L2 transform
            Eigen::Matrix4d T_L1_to_L2 = Eigen::Matrix4d::Identity();
            T_L1_to_L2(0, 3) = 0.0234207;
            T_L1_to_L2(2, 3) = 0.1105;
            T_L1_to_L2(1, 1) = 0;
            T_L1_to_L2(1, 2) = 1;
            T_L1_to_L2(2, 1) = -1;
            T_L1_to_L2(2, 2) = 0;
            
            Eigen::Matrix4d T_L2 = Eigen::Matrix4d::Identity();
            T_L2(0, 0) = cos(q2);
            T_L2(0, 1) = -sin(q2);
            T_L2(1, 0) = sin(q2);
            T_L2(1, 1) = cos(q2);
            
            T = T * T_L1_to_L2 * T_L2;
            
            if (joint_idx == 2) {
                // Joint 2 position (at L3 joint origin)
                Eigen::Matrix4d T_L2_to_L3 = Eigen::Matrix4d::Identity();
                T_L2_to_L3(1, 3) = -0.18;
                T = T * T_L2_to_L3;
            } else {
                // Include L3 transform
                Eigen::Matrix4d T_L2_to_L3 = Eigen::Matrix4d::Identity();
                T_L2_to_L3(1, 3) = -0.18;
                T_L2_to_L3(0, 0) = 0;
                T_L2_to_L3(0, 1) = 1;
                T_L2_to_L3(1, 0) = 1;
                T_L2_to_L3(1, 1) = 0;
                T_L2_to_L3(2, 2) = -1;
                
                Eigen::Matrix4d T_L3 = Eigen::Matrix4d::Identity();
                T_L3(0, 0) = cos(-q3);
                T_L3(0, 1) = -sin(-q3);
                T_L3(1, 0) = sin(-q3);
                T_L3(1, 1) = cos(-q3);
                
                T = T * T_L2_to_L3 * T_L3;
                
                if (joint_idx == 3) {
                    // Joint 3 position (at L4 joint origin)
                    Eigen::Matrix4d T_L3_to_L4 = Eigen::Matrix4d::Identity();
                    T_L3_to_L4(0, 3) = 0.0435;
                    T = T * T_L3_to_L4;
                }
            }
        }
        
        joint_pos = Eigen::Vector3d(T(0, 3), T(1, 3), T(2, 3));
    }
    
    // Vectors from joint to end-effector and target
    Eigen::Vector3d to_end = end_effector - joint_pos;
    Eigen::Vector3d to_target = target - joint_pos;
    
    // Check if vectors are too small
    double to_end_len = to_end.norm();
    double to_target_len = to_target.norm();
    
    if (to_end_len < 1e-6 || to_target_len < 1e-6) {
        return false;
    }
    
    // Normalize vectors
    to_end.normalize();
    to_target.normalize();
    
    // Compute rotation angle
    double dot_product = to_end.dot(to_target);
    dot_product = std::clamp(dot_product, -1.0, 1.0);
    double angle = acos(dot_product);
    
    if (angle < 1e-6) {
        return false;  // Already aligned
    }
    
    // Determine rotation axis based on URDF axis definitions
    Eigen::Vector3d joint_axis;
    double sign = 1.0;
    
    if (joint_idx == 0) {
        // Joint L1: axis="0 0 1" - rotates around world Z
        joint_axis = Eigen::Vector3d(0, 0, 1);
    } else if (joint_idx == 1) {
        // Joint L2: axis="0 0 1" in L2 frame (after rpy=-π/2 0 0 rotation and base rotation)
        // After base rotation, this is perpendicular to the arm plane
        joint_axis = Eigen::Vector3d(-sin(q1), cos(q1), 0);
    } else if (joint_idx == 2) {
        // Joint L3: axis="0 0 -1" in L3 frame - same direction as L2 in world frame
        joint_axis = Eigen::Vector3d(-sin(q1), cos(q1), 0);
    } else if (joint_idx == 3) {
        // Joint L4: axis="0 0 -1" in L4 frame - same direction
        joint_axis = Eigen::Vector3d(-sin(q1), cos(q1), 0);
    } else {
        joint_axis = Eigen::Vector3d(0, 0, 1);
    }
    
    Eigen::Vector3d cross = to_end.cross(to_target);
    sign = (cross.dot(joint_axis) > 0) ? 1.0 : -1.0;
    
    // Apply rotation with damping
    double damping = 0.4;
    double delta = sign * angle * damping;
    joint_angles[joint_idx] += delta;
    
    // Apply joint limits from URDF
    if (joint_idx == 0) {
        joint_angles[joint_idx] = std::clamp(joint_angles[joint_idx], -1.7, 1.7);
    } else if (joint_idx == 1) {
        joint_angles[joint_idx] = std::clamp(joint_angles[joint_idx], -0.98, 1.0);
    } else if (joint_idx == 2) {
        joint_angles[joint_idx] = std::clamp(joint_angles[joint_idx], -2.0, 1.3);
    } else if (joint_idx == 3) {
        joint_angles[joint_idx] = std::clamp(joint_angles[joint_idx], -2.0, 2.0);
    }
    
    return true;
}

void CCDNode::cyclicCoordinateDescent(double target_x, double target_y, double target_z)
{
    Eigen::Vector3d target(target_x, target_y, target_z);
    
    // Use current joint angles as starting point (not initial values)
    std::vector<double> joint_angles = {theta_1_, theta_2_, theta_3_, theta_4_, theta_5_, theta_6_};
    
    // Analytically solve for base joint angle (joint 0)
    // The base joint rotates around Z, so it should point towards the target in XY plane
    joint_angles[0] = atan2(target_y, target_x);
    joint_angles[0] = std::clamp(joint_angles[0], -1.7, 1.7);
    
    int iteration = 0;
    double error = std::numeric_limits<double>::max();
    double prev_error = error;
    int stall_count = 0;
    
    // Compute initial error
    Eigen::Vector3d initial_pos = forwardKinematics(joint_angles);
    error = (target - initial_pos).norm();
    
    RCLCPP_DEBUG(this->get_logger(), "Starting CCD: Initial pos=(%.3f, %.3f, %.3f), error=%.4f",
                initial_pos.x(), initial_pos.y(), initial_pos.z(), error);
    
    while (iteration < max_iterations_ && error > tolerance_) {
        // CCD works backwards from end-effector to base
        // For Parol6, we solve joints 1-3 (shoulder, elbow, wrist)
        // Joint 0 (base) is already solved analytically above
        for (int joint_idx = 3; joint_idx >= 1; joint_idx--) {
            bool changed = ccdIteration(target, joint_angles, joint_idx);
            if (!changed) {
                RCLCPP_DEBUG(this->get_logger(), "Joint %d did not change", joint_idx);
            }
        }
        
        // Compute error
        Eigen::Vector3d end_effector = forwardKinematics(joint_angles);
        error = (target - end_effector).norm();
        
        // Check for stalling
        if (std::abs(error - prev_error) < 1e-5) {
            stall_count++;
            if (stall_count > 15) {
                RCLCPP_DEBUG(this->get_logger(), "CCD stalled after %d iterations", iteration);
                break;  // Not making progress
            }
        } else {
            stall_count = 0;
        }
        prev_error = error;
        
        iteration++;
    }
    
    // Update joint angles
    theta_1_ = joint_angles[0];
    theta_2_ = joint_angles[1];
    theta_3_ = joint_angles[2];
    theta_4_ = joint_angles[3];
    // theta_5_ and theta_6_ remain as set by user or parameters
    
    Eigen::Vector3d final_pos = forwardKinematics(joint_angles);
    
    if (error <= tolerance_) {
        RCLCPP_INFO(this->get_logger(), 
                    "CCD converged in %d iterations. Target=(%.3f,%.3f,%.3f), Reached=(%.3f,%.3f,%.3f), Error=%.4f",
                    iteration, target_x, target_y, target_z, 
                    final_pos.x(), final_pos.y(), final_pos.z(), error);
    } else {
        RCLCPP_WARN(this->get_logger(), 
                    "CCD did not converge. Target=(%.3f,%.3f,%.3f), Reached=(%.3f,%.3f,%.3f), Error=%.4f after %d iterations",
                    target_x, target_y, target_z, 
                    final_pos.x(), final_pos.y(), final_pos.z(), error, iteration);
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Joint angles: L1=%.3f° L2=%.3f° L3=%.3f° L4=%.3f°",
                theta_1_*180/M_PI, theta_2_*180/M_PI, theta_3_*180/M_PI, theta_4_*180/M_PI);
}

void CCDNode::broadcastJointState()
{
    // Only run CCD if target has changed
    if (target_changed_) {
        cyclicCoordinateDescent(x_, y_, z_);
        target_changed_ = false;
        prev_x_ = x_;
        prev_y_ = y_;
        prev_z_ = z_;
    }
    
    // Always broadcast current joint state
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name = {"L1", "L2", "L3", "L4", "L5", "L6"};
    joint_state.position = {theta_1_, theta_2_, theta_3_, theta_4_, theta_5_, theta_6_};
    
    // Publish the joint state
    joint_state_publisher_->publish(joint_state);
}
