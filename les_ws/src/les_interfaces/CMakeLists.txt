cmake_minimum_required(VERSION 3.8)
project(les_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
# we will add the name of our custom interfaces here
 "msg/HardwareStatus.msg"
 "srv/SNameSNumber.srv"
 "srv/MyVec.srv"
 "action/TemplateA.action"
)
ament_export_dependencies(rosidl_default_runtime)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



ament_package()
