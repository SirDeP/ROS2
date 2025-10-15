cmake_minimum_required(VERSION 3.8)
project(les_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)   # nodig als er een actionserver node wordt gemaakt 

# find_package(templates_interfaces REQUIRED)   # gebruik maken van project messages 
 find_package(std_msgs REQUIRED)        # gebruik maken von standaard messages
find_package(geometry_msgs REQUIRED)
find_package(les_interfaces REQUIRED)


## 1 new line name of executalbe use the program name !
# add_executable(template_general src/templates/template_general.cpp) 
# add_executable(timer src/Timer.cpp)
# add_executable(template_walltimer src/templates/template_walltimer.cpp) 
# add_executable(walltimer src/walltimer.cpp) 
# add_executable(template_publisher src/templates/template_publisher.cpp) 
# add_executable(template_subscriber src/templates/template_subscriber.cpp) 
add_executable(pub_test src/publisher_test.cpp)
add_executable(sub_test src/subscriber_test.cpp)
add_executable(SerSen_test src/Service_Server_test.cpp)
add_executable(VectorService src/vector_service_server.cpp)
add_executable(VectorServiceClient src/vector_service_client.cpp)
add_executable(TempActionServer src/templates/template_actionserver.cpp)
add_executable(TempActionClient src/templates/template_actionclient.cpp)
#add_executable(template_serviceserver src/template_serviceserver.cpp) 
#add_executable(template_serviceclient src/template_serviceclient.cpp)
#add_executable(template_actionserver src/template_actionserver.cpp)
#add_executable(template_actionclient src/template_actionclient.cpp)
#

## 2 new line meaning that this program needs rclcpp
# ament_target_dependencies(template_general rclcpp  )
# ament_target_dependencies(timer rclcpp)
# ament_target_dependencies(template_walltimer rclcpp  ) 
# ament_target_dependencies(walltimer rclcpp  ) 
# ament_target_dependencies(template_publisher rclcpp std_msgs) 
# ament_target_dependencies(template_subscriber rclcpp std_msgs) 
ament_target_dependencies(pub_test rclcpp les_interfaces) # geometry_msgs
ament_target_dependencies(sub_test rclcpp les_interfaces)
ament_target_dependencies(SerSen_test rclcpp les_interfaces std_msgs)
ament_target_dependencies(VectorService rclcpp les_interfaces std_msgs)
ament_target_dependencies(VectorServiceClient rclcpp les_interfaces std_msgs)
# ament_target_dependencies(template_serviceserver rclcpp std_msgs templates_interfaces ) 
# ament_target_dependencies(template_serviceclient rclcpp std_msgs templates_interfaces ) 
ament_target_dependencies(TempActionServer rclcpp rclcpp_action les_interfaces ) 
ament_target_dependencies(TempActionClient rclcpp rclcpp_action les_interfaces ) 

## new 3 get 
install (TARGETS
# 	  template_general
#     timer
#     template_walltimer
#     walltimer
#     template_publisher
#     template_subscriber
    pub_test
    sub_test
    SerSen_test
    VectorService
    VectorServiceClient
    TempActionServer
    TempActionClient
#          template_serviceserver
#          template_serviceclient
#          template_actionserver
#          template_actionclient
          DESTINATION lib/${PROJECT_NAME}
          )



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




