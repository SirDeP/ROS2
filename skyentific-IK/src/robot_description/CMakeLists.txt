cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  tf2_ros
  geometry_msgs
  sensor_msgs
)


add_executable(ik_node src/IK-Node/main.cpp src/IK-Node/node/IK-node.cpp)
ament_target_dependencies(ik_node ${dependencies})

install(TARGETS
  ik_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
