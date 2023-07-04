cmake_minimum_required(VERSION 3.8)
project(vrx_localization)
find_package(ament_cmake REQUIRED)


# Python Scripts
install(PROGRAMS src/vrx_localization/robot_path_publisher.py
  DESTINATION lib/${PROJECT_NAME})

# Launch scripts
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Launch params
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params)

ament_python_install_package(
  vrx_localization
  PACKAGE_DIR src/vrx_localization
)
ament_package()