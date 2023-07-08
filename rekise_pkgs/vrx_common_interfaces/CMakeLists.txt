cmake_minimum_required(VERSION 3.8)
project(vrx_common_interfaces)

find_package(ament_cmake REQUIRED)

# https://roboticsbackend.com/ros2-create-custom-message/
# find_package(rosidl_generate_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PidDiagnose.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
