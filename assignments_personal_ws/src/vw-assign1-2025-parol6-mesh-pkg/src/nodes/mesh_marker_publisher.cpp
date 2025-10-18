#include "mesh_marker_publisher.hpp"

MeshMarkerPublisher::MeshMarkerPublisher() : Node("mesh_marker_publisher")
    {
        this->declare_parameter("board_location_x", 0.1);
        this->declare_parameter("board_location_y", 0.0);
        this->declare_parameter("board_location_z", 0.0);

        this->declare_parameter("coin_holder_location_x", 0.30);
        this->declare_parameter("coin_holder_location_y", 0.08);
        this->declare_parameter("coin_holder_location_z", 0.0);

        this->declare_parameter("coin_location_x", 0.34);
        this->declare_parameter("coin_location_y", 0.08);
        this->declare_parameter("coin_location_z", 0.0);

        board_location_x_ = this->get_parameter("board_location_x").as_double();
        board_location_y_ = this->get_parameter("board_location_y").as_double();
        board_location_z_ = this->get_parameter("board_location_z").as_double();

        coin_holder_location_x_ = this->get_parameter("coin_holder_location_x").as_double();
        coin_holder_location_y_ = this->get_parameter("coin_holder_location_y").as_double();
        coin_holder_location_z_ = this->get_parameter("coin_holder_location_z").as_double();

        coin_location_x_ = this->get_parameter("coin_location_x").as_double();
        coin_location_y_ = this->get_parameter("coin_location_y").as_double();
        coin_location_z_ = this->get_parameter("coin_location_z").as_double();

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MeshMarkerPublisher::publish_marker, this));
    }


void MeshMarkerPublisher::publish_marker()
{
    publish_coin();
    publish_coin_holder();
    publish_board();
    publish_board_base();
}

void MeshMarkerPublisher::publish_coin()
{
    auto coin = visualization_msgs::msg::Marker();
    coin.header.frame_id = "world";
    coin.mesh_resource = "package://vw-assign1-2025-parol6-mesh-pkg/meshes/muntje.STL";
    coin.mesh_use_embedded_materials = true;

    coin.header.stamp = this->now();
    coin.ns = "mesh";
    coin.id = 0;

    coin.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    coin.action = visualization_msgs::msg::Marker::ADD;

    coin.pose.position.x = coin_location_x_;
    coin.pose.position.y = coin_location_y_;
    coin.pose.position.z = coin_location_z_;
    coin.pose.orientation.x = 0.0; // Rotation in quaternions
    coin.pose.orientation.y = 0.0;
    coin.pose.orientation.z = 0.0;
    coin.pose.orientation.w = 1.0;

    coin.scale.x = 0.001;
    coin.scale.y = 0.001;
    coin.scale.z = 0.001;

    coin.color.r = 0.0;
    coin.color.g = 1.0;
    coin.color.b = 0.0;
    coin.color.a = 1.0; // Don't forget to set the alpha!

    marker_pub_->publish(coin);
}

void MeshMarkerPublisher::publish_coin_holder()
{
    auto coin_holder = visualization_msgs::msg::Marker();
    coin_holder.header.frame_id = "world";
    coin_holder.mesh_resource = "package://vw-assign1-2025-parol6-mesh-pkg/meshes/muntjes_houder.STL";
    coin_holder.mesh_use_embedded_materials = true;

    coin_holder.header.stamp = this->now();
    coin_holder.ns = "mesh";
    coin_holder.id = 1;

    coin_holder.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    coin_holder.action = visualization_msgs::msg::Marker::ADD;

    coin_holder.pose.position.x = coin_holder_location_x_;
    coin_holder.pose.position.y = coin_holder_location_y_;
    coin_holder.pose.position.z = coin_holder_location_z_;
    coin_holder.pose.orientation.x = 0.7071068; // Rotation in quaternions
    coin_holder.pose.orientation.y = 0.0;
    coin_holder.pose.orientation.z = 0.0;
    coin_holder.pose.orientation.w = 0.7071068;

    coin_holder.scale.x = 0.001;
    coin_holder.scale.y = 0.001;
    coin_holder.scale.z = 0.001;

    coin_holder.color.r = 1.0;
    coin_holder.color.g = 1.0;
    coin_holder.color.b = 1.0;
    coin_holder.color.a = 1.0; // Don't forget to set the alpha!

    marker_pub_->publish(coin_holder);
}

void MeshMarkerPublisher::publish_board()
{
    auto board = visualization_msgs::msg::Marker();
    board.header.frame_id = "world";
    board.mesh_resource = "package://vw-assign1-2025-parol6-mesh-pkg/meshes/board_conn_4.STL";
    board.mesh_use_embedded_materials = true;

    board.header.stamp = this->now();
    board.ns = "mesh";
    board.id = 2;

    board.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    board.action = visualization_msgs::msg::Marker::ADD;

    board.pose.position.x = board_location_x_ + 0.281; // +0.281
    board.pose.position.y = board_location_y_ + 0.031; // +0.031
    board.pose.position.z = board_location_z_ + 0.006;
    board.pose.orientation.x = 0.0;     // Rotation in quaternions
    board.pose.orientation.y = 0.0;
    board.pose.orientation.z = -0.1127597;
    board.pose.orientation.w = 0.9936223;

    board.scale.x = 0.001;
    board.scale.y = 0.001;
    board.scale.z = 0.001;

    board.color.r = 0.0;
    board.color.g = 0.0;
    board.color.b = 1.0;
    board.color.a = 1.0; // Don't forget to set the alpha!

    marker_pub_->publish(board);
}

void MeshMarkerPublisher::publish_board_base()
{
    auto board_base = visualization_msgs::msg::Marker();
    board_base.header.frame_id = "world";
    board_base.mesh_resource = "package://vw-assign1-2025-parol6-mesh-pkg/meshes/base_conn_4.STL";
    board_base.mesh_use_embedded_materials = true;

    board_base.header.stamp = this->now();
    board_base.ns = "mesh";
    board_base.id = 3;

    board_base.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    board_base.action = visualization_msgs::msg::Marker::ADD;

    board_base.pose.position.x = board_location_x_;
    board_base.pose.position.y = board_location_y_;
    board_base.pose.position.z = board_location_z_;
    board_base.pose.orientation.x = 0.0; // Rotation in quaternions
    board_base.pose.orientation.y = 0.0;
    board_base.pose.orientation.z = 0.0;
    board_base.pose.orientation.w = 1.0;

    board_base.scale.x = 0.001;
    board_base.scale.y = 0.001;
    board_base.scale.z = 0.001;

    board_base.color.r = 1.0;
    board_base.color.g = 1.0;
    board_base.color.b = 1.0;
    board_base.color.a = 1.0; // Don't forget to set the alpha!

    marker_pub_->publish(board_base);
}

