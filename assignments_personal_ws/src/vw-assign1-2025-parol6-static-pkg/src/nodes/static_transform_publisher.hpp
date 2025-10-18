#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticTfNode : public rclcpp::Node {
public:
    StaticTfNode();
private:
  geometry_msgs::msg::TransformStamped createTransform(
      const std::string& parent, const std::string& child,
      double x, double y, double z, double roll, double pitch, double yaw);

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};
