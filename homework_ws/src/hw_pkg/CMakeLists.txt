cmake_minimum_required(VERSION 3.8)
project(hw_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(std_msgs REQUIRED)
find_package(hw_interfaces)

# add_executable(clock src/Hw1/clock.cpp)
# add_executable(hw2ex2 src/Hw1/hw2_ex2.cpp)
# add_executable(hw3_4ex1 src/Hw1/hw3_4_ex1.cpp)
# add_executable(hw3_4ex2 src/Hw1/hw3_4_ex2.cpp)
# add_executable(hw5_Pub src/Hw1/hw5_Pub.cpp)
# add_executable(hw5_Sub src/Hw1/hw5_Sub.cpp)
add_executable(hw5_6ex1 src/HW2/DerivativeServiceServer.cpp)
add_executable(hw5_6ex2 src/HW2/DerivativeServiceClient.cpp)
add_executable(hw5_6ex3 src/HW2/DerivativeServiceClientEX3.cpp)

# ament_target_dependencies(clock rclcpp)
# ament_target_dependencies(hw2ex2 rclcpp)
# ament_target_dependencies(hw3_4ex1 rclcpp std_msgs)
# ament_target_dependencies(hw3_4ex2 rclcpp std_msgs)
# ament_target_dependencies(hw5_Pub rclcpp hw_interfaces)
# ament_target_dependencies(hw5_Sub rclcpp hw_interfaces)
ament_target_dependencies(hw5_6ex1 rclcpp hw_interfaces)
ament_target_dependencies(hw5_6ex2 rclcpp hw_interfaces)
ament_target_dependencies(hw5_6ex3 rclcpp hw_interfaces)

install (TARGETS
          # clock
          # hw2ex2
          # hw3_4ex1
          # hw3_4ex2
          # hw5_Pub
          # hw5_Sub
          hw5_6ex1
          hw5_6ex2
          hw5_6ex3
          DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
