cmake_minimum_required(VERSION 3.8)
project(vrx_control)
find_package(ament_cmake REQUIRED)

# Python scripts
install(PROGRAMS
  src/${PROJECT_NAME}/yaw_controller.py
  src/${PROJECT_NAME}/absolute_heading_controller.py
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
