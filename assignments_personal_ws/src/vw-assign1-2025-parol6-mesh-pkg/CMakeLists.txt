cmake_minimum_required(VERSION 3.8)
project(vw-assign1-2025-parol6-mesh-pkg)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp)
find_package(visualization_msgs)

add_executable(mesh_pub src/mesh_marker_publisher.cpp)

ament_target_dependencies(mesh_pub rclcpp visualization_msgs)

install(TARGETS
  mesh_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
