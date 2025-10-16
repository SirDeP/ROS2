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

add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp tf2 tf2_ros geometry_msgs)


install(TARGETS
  pub
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
