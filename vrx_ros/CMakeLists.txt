cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(vrx_ros)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(optical_frame_publisher src/optical_frame_publisher.cc)
ament_target_dependencies(optical_frame_publisher
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
)

add_executable(pose_tf_broadcaster src/pose_tf_broadcaster.cc)
ament_target_dependencies(pose_tf_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_msgs
)

# Resources
install(TARGETS
  optical_frame_publisher
  pose_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
