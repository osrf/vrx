cmake_minimum_required(VERSION 3.8)
project(vrx_control)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# cpp programs
# add_executable(even_parameters_node src/even_parameters_node.cc)
# ament_target_dependencies(even_parameters_node
  # rclcpp
# )



# Install cpp binaries
# install(TARGETS
#   even_parameters_node
#   DESTINATION lib/${PROJECT_NAME})

# Python scripts
install(PROGRAMS
  src/${PROJECT_NAME}/yaw_controller.py
  src/${PROJECT_NAME}/absolute_heading_controller.py
  src/${PROJECT_NAME}/joy_command_consolidator.py
  src/${PROJECT_NAME}/forward_velocity_controller.py
  src/${PROJECT_NAME}/forward_velocity_controller.py
  src/${PROJECT_NAME}/yaw_rate_controller.py
  src/${PROJECT_NAME}/utils.py
  src/${PROJECT_NAME}/angles_in_rpy.py
  DESTINATION lib/${PROJECT_NAME}
)

#============================================================================
# Resources
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# Install python modules
ament_python_install_package(
    ${PROJECT_NAME}    
    PACKAGE_DIR src/${PROJECT_NAME}
)


ament_package()
