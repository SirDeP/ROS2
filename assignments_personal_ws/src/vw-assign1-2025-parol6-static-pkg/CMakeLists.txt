cmake_minimum_required(VERSION 3.8)
project(vw-assign1-2025-parol6-static-pkg)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(vw-assign1-2025-static_transform_publisher-node
    src/vw-assign1-static_transform_publisher-node/main.cpp
    src/vw-assign1-static_transform_publisher-node/nodes/static_transform_publisher.cpp
)

ament_target_dependencies(vw-assign1-2025-static_transform_publisher-node rclcpp tf2 tf2_ros geometry_msgs)


install(TARGETS
  vw-assign1-2025-static_transform_publisher-node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
