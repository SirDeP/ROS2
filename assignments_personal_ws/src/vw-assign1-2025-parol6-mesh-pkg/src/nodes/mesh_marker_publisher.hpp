#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MeshMarkerPublisher : public rclcpp::Node
{

public:
    MeshMarkerPublisher();
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int marker_x_ = 0;

    double board_location_x_;
    double board_location_y_;
    double board_location_z_;

    double coin_holder_location_x_;
    double coin_holder_location_y_;
    double coin_holder_location_z_;

    double coin_location_x_;
    double coin_location_y_;
    double coin_location_z_;

    void publish_marker();

    void publish_coin();
    void publish_coin_holder();
    void publish_board();
    void publish_board_base();
};
