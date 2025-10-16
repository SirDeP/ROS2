#include <memory>
#include <string>
#include <fstream>
#include <streambuf>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "robot_state_publisher/robot_state_publisher.hpp"
#include "urdf/model.h"

class ObjectPublisherNode : public rclcpp::Node {
public:
    ObjectPublisherNode()
    : Node("object_publisher_node") {
        // 1. Load URDF file
        std::string urdf_file = this->declare_parameter<std::string>("urdf_file", "test.urdf.xacro");
        std::string urdf_path = "package://vw-assign1-2025-parol6-static-pkg/urdf/" + urdf_file;

        std::ifstream file(urdf_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", urdf_path.c_str());
            rclcpp::shutdown();
            return;
        }

        std::string urdf_xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        file.close();

        // 2. Parse URDF
        urdf::Model urdf_model;
        if (!urdf_model.initString(urdf_xml)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            rclcpp::shutdown();
            return;
        }

        // 3. Robot State Publisher
        auto state_publisher_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(urdf_model);
        joint_state_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                state_publisher_->publishFixedTransforms(this->get_clock()->now());
            }
        );

        // 4. Static Transform from world → object_base
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "world";
        static_transform.child_frame_id = "object_base";

        static_transform.transform.translation.x = 1.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;

        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;

        broadcaster_->sendTransform(static_transform);

        RCLCPP_INFO(this->get_logger(), "ObjectPublisherNode started.");
    }

private:
    std::shared_ptr<robot_state_publisher::RobotStatePublisher> state_publisher_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
