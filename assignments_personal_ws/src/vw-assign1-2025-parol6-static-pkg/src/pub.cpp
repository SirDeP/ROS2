#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticTfNode : public rclcpp::Node {
public:
  StaticTfNode() : Node("connect4_static_tf") {
    double x = declare_parameter<double>("x", 0.60);
    double y = declare_parameter<double>("y", 0.00);
    double z = declare_parameter<double>("z", 0.00);
    double roll  = declare_parameter<double>("roll",  0.0);
    double pitch = declare_parameter<double>("pitch", 0.0);
    double yaw   = declare_parameter<double>("yaw",   0.0);

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // world -> base_conn_4
    transforms.push_back(createTransform("world", "base_conn_4", x, y, z, roll, pitch, yaw));

    broadcaster_->sendTransform(transforms);

    RCLCPP_INFO(get_logger(), "Published %zu static transforms for Connect4", transforms.size());
  }

private:
  geometry_msgs::msg::TransformStamped createTransform(
      const std::string& parent, const std::string& child,
      double x, double y, double z, double roll, double pitch, double yaw) {

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTfNode>());
  rclcpp::shutdown();
  return 0;
}
