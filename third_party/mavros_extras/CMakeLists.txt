cmake_minimum_required(VERSION 3.5)
project(mavros_extras)

# This package intentionally contains only the unchanged upstream MAVROS 2.14
# HIL plugin. MAVROS itself remains a binary dependency and owns transport,
# framing, CRC, and the plugin ABI.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(message_filters REQUIRED)
find_package(mavlink REQUIRED)
find_package(mavros REQUIRED)
find_package(libmavconn REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(mavros_extras_plugins SHARED src/plugins/hil.cpp)
target_include_directories(mavros_extras_plugins PUBLIC
  ${mavlink_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${libmavconn_INCLUDE_DIRS}
  ${mavros_INCLUDE_DIRS})
target_link_libraries(mavros_extras_plugins PUBLIC
  ${geographic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mavros_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${trajectory_msgs_TARGETS}
  angles::angles
  diagnostic_updater::diagnostic_updater
  geographic_msgs::validation
  message_filters::message_filters
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rcpputils::rcpputils
  sensor_msgs::sensor_msgs_library
  tf2_eigen::tf2_eigen
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  ${mavros_LIBRARIES})

pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)
install(TARGETS mavros_extras_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_package()
