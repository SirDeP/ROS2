#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
class JointPublisherNode : public rclcpp::Node
{
public:
    JointPublisherNode() : Node("ik_node")
    {
        // Declare and get parameters
        // this->declare_parameter<double>("theta_1", 0.0);
        // this->declare_parameter<double>("theta_2", 0.0);
        // this->declare_parameter<double>("theta_3", 0.0);
        // this->declare_parameter<double>("theta_4", 0.0);
        // this->declare_parameter<double>("theta_5", 0.0);
        // this->declare_parameter<double>("theta_6", 0.0);
        // theta_1_ = this->get_parameter("theta_1").as_double();
        // theta_2_ = this->get_parameter("theta_2").as_double();
        // theta_3_ = this->get_parameter("theta_3").as_double();
        // theta_4_ = this->get_parameter("theta_4").as_double();
        // theta_5_ = this->get_parameter("theta_5").as_double();
        // theta_6_ = this->get_parameter("theta_6").as_double();

        this->declare_parameter<double>("x", 0.3);
        this->declare_parameter<double>("y", 0.0);
        this->declare_parameter<double>("z", 0.15);
        this->declare_parameter<double>("base_height", 0.081); // PAROL6 base height
        this->declare_parameter<double>("L1", 0.18);           // PAROL6 upper arm length
        this->declare_parameter<double>("L2", 0.17635);        // PAROL6 forearm length
        this->declare_parameter<double>("L3", 0.0435);         // PAROL6 elbow offset
        x_ = this->get_parameter("x").as_double();
        y_ = this->get_parameter("y").as_double();
        z_ = this->get_parameter("z").as_double();
        L1_ = this->get_parameter("L1").as_double();
        L2_ = this->get_parameter("L2").as_double();
        L3_ = this->get_parameter("L3").as_double();
        base_height_ = this->get_parameter("base_height").as_double();

        // Calculate initial joint angles
        inversekinematics();

        // Register parameter callback for dynamic updates
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&JointPublisherNode::onParameterChange, this, std::placeholders::_1));

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

private:
    void broadcastJointState()
    {
        // Create a JointState message
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name = {"L1", "L2", "L3", "L4", "L5", "L6"};
        joint_state.position = {theta_1_, theta_2_, theta_3_, theta_4_, theta_5_, theta_6_};
        // Publish the joint state
        joint_state_publisher_->publish(joint_state);
    }
    double theta_1_;
    double theta_2_;
    double theta_3_;
    double theta_4_;
    double theta_5_;
    double theta_6_;

    double x_, y_, z_, x2_, y2_, L1_, L2_, L3_, base_height_;
    double theta1_, theta2_, theta3_;
    double costheta2_, sintheta2_;
    double k1_, k2_;

    void inversekinematics()
    {
        // Step 1: Base rotation (Joint 1)
        theta_1_ = atan2(y_, x_);

        // Step 2: Project to 2D plane (r-z plane)
        double r = sqrt(x_ * x_ + y_ * y_); // radial distance
        double z_adj = z_ - base_height_;   // adjust for base height

        // Step 3: 2-link planar IK in r-z plane
        double d = sqrt(r * r + z_adj * z_adj);

        costheta2_ = (d * d - L1_ * L1_ - L2_ * L2_) / (2 * L1_ * L2_);
        sintheta2_ = sqrt(1 - costheta2_ * costheta2_); // elbow-up
        theta2_ = atan2(sintheta2_, costheta2_);

        k1_ = L1_ + L2_ * costheta2_;
        k2_ = L2_ * sintheta2_;
        theta1_ = atan2(z_adj, r) - atan2(k2_, k1_); // Use z_adj and r, not y_ and x_

        // Step 4: Map to PAROL6 joints (accounting for URDF offsets)
        theta_2_ = theta1_ - (M_PI / 2);
        theta_3_ = theta2_;
        theta_4_ = -theta1_ - theta2_ - (M_PI / 2); // wrist compensation
        theta_5_ = theta_1_;
        theta_6_ = 0.00091;
        RCLCPP_INFO(this->get_logger(), "Joint states (%f %f %f %f %f %f) were sent to the system.",
                    theta_1_, theta_2_, theta_3_, theta_4_,
                    theta_5_, theta_6_);
    }

    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "x")
            {
                x_ = parameter.as_double();
            }
            else if (parameter.get_name() == "y")
            {
                y_ = parameter.as_double();
            }
            else if (parameter.get_name() == "z")
            {
                z_ = parameter.as_double();
            }
            else if (parameter.get_name() == "base_height")
            {
                base_height_ = parameter.as_double();
            }
            else if (parameter.get_name() == "L1")
            {
                L1_ = parameter.as_double();
            }
            else if (parameter.get_name() == "L2")
            {
                L2_ = parameter.as_double();
            }
            else if (parameter.get_name() == "L3")
            {
                L3_ = parameter.as_double();
            }
        }
        inversekinematics();
        RCLCPP_INFO(this->get_logger(), "Updated parameters: x=%f, y=%f, L1=%f, L2=%f, L3=%f", x_, y_, L1_, L2_, L3_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";
        return result;
    }

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <cmath>

// /**
//  * @brief Inverse Kinematics Node for Parol6 Robot
//  *
//  * This node calculates joint angles for the Parol6 6-DOF robot arm
//  * based on a target (x, y, z) position in Cartesian coordinates.
//  *
//  * URDF Analysis - Parol6 Robot Structure:
//  * - Joint L1: Base rotation around Z-axis (vertical)
//  * - Joint L2: Shoulder pitch, origin at z=0.1105m, rotated -90° around X
//  * - Joint L3: Elbow pitch, origin at y=-0.18m from shoulder
//  * - Joint L4: Wrist 1, origin at x=0.0435m, rotated 90° around X
//  * - Joint L5: Wrist 2, origin at z=-0.17635m
//  * - Joint L6: Wrist 3 (end effector rotation)
//  */
// class IKNode : public rclcpp::Node
// {
// public:
//     IKNode() : Node("ik_node")
//     {
//         // Declare and get parameters for target position
//         this->declare_parameter<double>("target_x", 0.3);
//         this->declare_parameter<double>("target_y", 0.0);
//         this->declare_parameter<double>("target_z", 0.2);
//         this->declare_parameter<int>("case_number", 1);

//         target_x_ = this->get_parameter("target_x").as_double();
//         target_y_ = this->get_parameter("target_y").as_double();
//         target_z_ = this->get_parameter("target_z").as_double();
//         case_num_ = this->get_parameter("case_number").as_int();

//         param_callback_handle_ = this->add_on_set_parameters_callback(
//             std::bind(&IKNode::onParameterChange, this, std::placeholders::_1));

//         // Define link lengths for Parol6 robot (in meters) from URDF
//         // From URDF analysis:
//         // - Joint L2 origin: xyz="0.0234207210610375 0 0.1105"
//         // - Joint L3 origin: xyz="0 -0.18 0"
//         // - Joint L4 origin: xyz="0.0435 0 0"
//         // - Joint L5 origin: xyz="0 0 -0.17635"
//         base_height_ = 0.1105;  // Height from base_link to shoulder joint
//         L1_ = 0.18;             // Upper arm length (shoulder to elbow)
//         L2_ = 0.17635;          // Forearm length (elbow to wrist)
//         elbow_offset_ = 0.0435; // Elbow offset in X direction

//         // Select predefined case or use custom x,y,z
//         if (case_num_ > 0) {
//             selectCase(case_num_);
//         }

//         // Calculate inverse kinematics
//         bool success = calculateIK();

//         if (!success) {
//             RCLCPP_ERROR(this->get_logger(), "IK calculation failed! Target unreachable.");
//             // Set to safe home position on failure
//             joint_L1_ = 0.0;
//             joint_L2_ = 0.0;
//             joint_L3_ = 0.0;
//             joint_L4_ = 0.0;
//             joint_L5_ = 0.0;
//             joint_L6_ = 0.0;
//         }

//         RCLCPP_INFO(this->get_logger(),
//                     "Target position: x=%.3f, y=%.3f, z=%.3f",
//                     target_x_, target_y_, target_z_);
//         RCLCPP_INFO(this->get_logger(),
//                     "Joint angles (deg): L1=%.2f, L2=%.2f, L3=%.2f, L4=%.2f, L5=%.2f, L6=%.2f",
//                     joint_L1_ * 180.0/M_PI, joint_L2_ * 180.0/M_PI, joint_L3_ * 180.0/M_PI,
//                     joint_L4_ * 180.0/M_PI, joint_L5_ * 180.0/M_PI, joint_L6_ * 180.0/M_PI);

//         // Initialize the joint state publisher
//         joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

//         // Initialize the transform buffer and listener
//         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         while(true) {
//             target_x_ = fmod(target_x_ + 0.001, 0.25);
//             target_y_ = fmod(target_y_ + 0.001, 0.25);
//             target_z_ = fmod(target_z_ + 0.001, 0.25);
//             calculateIK();
//             broadcastJointState();
//             std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         }

//         // Set up a timer to periodically publish joint states
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),
//             std::bind(&IKNode::broadcastJointState, this));
//     }

// private:
//     // Target position in Cartesian coordinates
//     double target_x_, target_y_, target_z_;
//     int case_num_;

//     // Robot link lengths (from URDF)
//     double base_height_;  // Height from base to shoulder joint
//     double L1_;           // Upper arm length
//     double L2_;           // Forearm length
//     double elbow_offset_; // Elbow offset

//     // Joint angles for Parol6 (L1-L6)
//     double joint_L1_;  // Base rotation
//     double joint_L2_;  // Shoulder pitch
//     double joint_L3_;  // Elbow pitch
//     double joint_L4_;  // Wrist 1
//     double joint_L5_;  // Wrist 2
//     double joint_L6_;  // Wrist 3

//     rcl_interfaces::msg::SetParametersResult onParameterChange(
//         const std::vector<rclcpp::Parameter> & parameters)
//     {
//         bool needs_recalc = false;

//         for (const auto & parameter : parameters)
//         {
//             if (parameter.get_name() == "target_x")
//             {
//                 target_x_ = parameter.as_double();
//                 needs_recalc = true;
//             }
//             else if (parameter.get_name() == "target_y")
//             {
//                 target_y_ = parameter.as_double();
//                 needs_recalc = true;
//             }
//             else if (parameter.get_name() == "target_z")
//             {
//                 target_z_ = parameter.as_double();
//                 needs_recalc = true;
//             }
//             else if (parameter.get_name() == "case_number")
//             {
//                 case_num_ = parameter.as_int();
//                 if (case_num_ > 0) {
//                     selectCase(case_num_);
//                 }
//                 needs_recalc = true;
//             }
//         }

//         if (needs_recalc) {
//             bool success = calculateIK();
//             if (!success) {
//                 RCLCPP_ERROR(this->get_logger(), "IK calculation failed! Target unreachable.");
//             } else {
//                 RCLCPP_INFO(this->get_logger(),
//                             "Updated target: x=%.3f, y=%.3f, z=%.3f -> Joints: L1=%.2f°, L2=%.2f°, L3=%.2f°",
//                             target_x_, target_y_, target_z_,
//                             joint_L1_ * 180.0/M_PI, joint_L2_ * 180.0/M_PI, joint_L3_ * 180.0/M_PI);
//             }
//         }

//         rcl_interfaces::msg::SetParametersResult result;
//         result.successful = true;
//         result.reason = "";
//         return result;
//     }

//     /**
//      * @brief Select a predefined test case position
//      * @param case_num Case number (1-6)
//      */
//     void selectCase(int case_num)
//     {
//         // Predefined test cases within Parol6 workspace
//         // Workspace: approximately 0.1m to 0.35m reach, height 0 to 0.4m
//         switch(case_num) {
//             case 1:
//                 // Forward reach at mid-height
//                 target_x_ = 0.25; target_y_ = 0.0; target_z_ = 0.20;
//                 RCLCPP_INFO(this->get_logger(), "Case 1: Forward mid-height position");
//                 break;
//             case 2:
//                 // Right side reach
//                 target_x_ = 0.20; target_y_ = 0.15; target_z_ = 0.18;
//                 RCLCPP_INFO(this->get_logger(), "Case 2: Right side reach");
//                 break;
//             case 3:
//                 // Left side reach (higher)
//                 target_x_ = 0.20; target_y_ = -0.15; target_z_ = 0.25;
//                 RCLCPP_INFO(this->get_logger(), "Case 3: Left side high reach");
//                 break;
//             case 4:
//                 // Near vertical reach (arm mostly up)
//                 target_x_ = 0.10; target_y_ = 0.0; target_z_ = 0.35;
//                 RCLCPP_INFO(this->get_logger(), "Case 4: Near vertical reach");
//                 break;
//             case 5:
//                 // Low forward reach
//                 target_x_ = 0.30; target_y_ = 0.0; target_z_ = 0.12;
//                 RCLCPP_INFO(this->get_logger(), "Case 5: Low forward reach");
//                 break;
//             case 6:
//                 // Diagonal reach
//                 target_x_ = 0.18; target_y_ = 0.18; target_z_ = 0.22;
//                 RCLCPP_INFO(this->get_logger(), "Case 6: Diagonal reach");
//                 break;
//             default:
//                 RCLCPP_WARN(this->get_logger(), "Unknown case number %d, using provided x,y,z", case_num);
//                 break;
//         }
//     }

//     /**
//      * @brief Calculate inverse kinematics for Parol6 robot
//      *
//      * Uses geometric approach based on the assignment formulas:
//      * 1. Calculate cos(θ2) from the target distance
//      * 2. Calculate sin(θ2) and θ2 using atan2
//      * 3. Calculate k1, k2 and use them to find θ1
//      * 4. Apply Parol6-specific joint angle transformations
//      *
//      * @return true if solution found within joint limits, false otherwise
//      */
//     bool calculateIK()
//     {
//         double x = target_x_;
//         double y = target_y_;
//         double z = target_z_;

//         // Step 1: Calculate base rotation (Joint L1) - rotation around Z axis
//         // θ0 = atan2(y, x)
//         double theta0 = std::atan2(y, x);

//         // Step 2: Project to 2D plane for the arm kinematics
//         // Calculate radial distance in XY plane
//         double r_xy = std::sqrt(x*x + y*y);

//         // Adjust z to account for base height (shoulder is at base_height_)
//         double z_adj = z - base_height_;

//         // Step 3: Check workspace reachability
//         // The arm works in the r-z plane after base rotation
//         double r_planar = std::sqrt(r_xy*r_xy + z_adj*z_adj);

//         double max_reach = L1_ + L2_;
//         double min_reach = std::abs(L1_ - L2_);

//         if (r_planar > max_reach) {
//             RCLCPP_ERROR(this->get_logger(),
//                         "Target too far! Distance: %.4f m, Max reach: %.4f m",
//                         r_planar, max_reach);
//             return false;
//         }

//         if (r_planar < min_reach + 0.01) {  // Small margin for numerical stability
//             RCLCPP_ERROR(this->get_logger(),
//                         "Target too close! Distance: %.4f m, Min reach: %.4f m",
//                         r_planar, min_reach);
//             return false;
//         }

//         // Step 4: Calculate elbow angle (θ2) using law of cosines
//         // cos(θ2) = (r² - L1² - L2²) / (2*L1*L2)
//         double cos_theta2 = (r_planar*r_planar - L1_*L1_ - L2_*L2_) / (2.0 * L1_ * L2_);

//         // Clamp to valid range for numerical stability
//         cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2));

//         // sin(θ2) = sqrt(1 - cos²(θ2))
//         // Using positive sqrt gives elbow-up configuration
//         double sin_theta2 = std::sqrt(1.0 - cos_theta2*cos_theta2);

//         // θ2 = atan2(sin(θ2), cos(θ2))
//         double theta2 = std::atan2(sin_theta2, cos_theta2);

//         // Step 5: Calculate k1 and k2 for shoulder angle
//         // k1 = L1 + L2*cos(θ2)
//         // k2 = L2*sin(θ2)
//         double k1 = L1_ + L2_ * cos_theta2;
//         double k2 = L2_ * sin_theta2;

//         // Step 6: Calculate shoulder angle (θ1)
//         // θ1 = atan2(z_adj, r_xy) - atan2(k2, k1)
//         double theta1 = std::atan2(z_adj, r_xy) - std::atan2(k2, k1);

//         // Step 7: Transform to Parol6 joint angles
//         // From the assignment, adapted for Parol6 URDF:
//         //
//         // URDF Analysis:
//         // - Joint L1: Rotates around Z, axis="0 0 1"
//         // - Joint L2: Origin rpy="-π/2 0 0", axis="0 0 1" -> effectively pitch
//         // - Joint L3: Origin rpy="π 0 -π/2", axis="0 0 -1" -> elbow pitch (inverted)
//         // - Joint L4: Origin rpy="π/2 0 π", axis="0 0 -1" -> wrist pitch
//         // - Joint L5: Origin rpy="-π/2 0 0", axis="0 0 -1"
//         // - Joint L6: Origin rpy="π/2 0 0", axis="0 0 -1"
//         //
//         // Based on the assignment transformations:
//         // θ0 = θ0 (base rotation)
//         // θ1 = θ1 - π/2 (shoulder offset due to URDF orientation)
//         // θ2 = θ2 (elbow)
//         // θ3 = -θ1 - θ2 - π/2 (wrist compensation to keep end effector orientation)
//         // θ4 = θ0 (wrist rotation matches base)
//         // θ5 = small offset

//         joint_L1_ = theta0;                              // Base rotation
//         joint_L2_ = theta1 - M_PI_2;                     // Shoulder with -90° offset
//         joint_L3_ = theta2;                              // Elbow angle
//         joint_L4_ = -theta1 - theta2 - M_PI_2;           // Wrist 1 compensation
//         joint_L5_ = -theta0;                             // Wrist 2
//         joint_L6_ = 0.0091;                              // End effector (small offset)

//         // Step 8: Check joint limits from URDF
//         bool within_limits = true;

//         // Joint L1: lower="-1.7" upper="1.7"
//         if (joint_L1_ < -1.7 || joint_L1_ > 1.7) {
//             RCLCPP_WARN(this->get_logger(), "Joint L1 out of limits: %.3f rad (limits: ±1.7)", joint_L1_);
//             within_limits = false;
//         }

//         // Joint L2: lower="-0.98" upper="1.0"
//         if (joint_L2_ < -0.98 || joint_L2_ > 1.0) {
//             RCLCPP_WARN(this->get_logger(), "Joint L2 out of limits: %.3f rad (limits: -0.98 to 1.0)", joint_L2_);
//             within_limits = false;
//         }

//         // Joint L3: lower="-2" upper="1.3" with axis="0 0 -1" (inverted)
//         if (joint_L3_ < -2.0 || joint_L3_ > 1.3) {
//             RCLCPP_WARN(this->get_logger(), "Joint L3 out of limits: %.3f rad (limits: -2.0 to 1.3)", joint_L3_);
//             within_limits = false;
//         }

//         // Joint L4: lower="-2" upper="2"
//         if (joint_L4_ < -2.0 || joint_L4_ > 2.0) {
//             RCLCPP_WARN(this->get_logger(), "Joint L4 out of limits: %.3f rad (limits: ±2.0)", joint_L4_);
//             within_limits = false;
//         }

//         // Joint L5: lower="-2.1" upper="2.1"
//         if (joint_L5_ < -2.1 || joint_L5_ > 2.1) {
//             RCLCPP_WARN(this->get_logger(), "Joint L5 out of limits: %.3f rad (limits: ±2.1)", joint_L5_);
//             within_limits = false;
//         }

//         return within_limits;
//     }

//     /**
//      * @brief Publish current joint state to /joint_states topic
//      */
//     void broadcastJointState()
//     {
//         // Create a JointState message
//         auto joint_state = sensor_msgs::msg::JointState();
//         joint_state.header.stamp = this->get_clock()->now();

//         // Parol6 joint names (from URDF)
//         joint_state.name = {"L1", "L2", "L3", "L4", "L5", "L6"};
//         joint_state.position = {joint_L1_, joint_L2_, joint_L3_, joint_L4_, joint_L5_, joint_L6_};

//         // Publish the joint state
//         joint_state_publisher_->publish(joint_state);
//     }

//     // ROS2 handles
//     OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
//     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<IKNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }