cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(vrx_gazebo)
find_package(ament_cmake REQUIRED)

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# Models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

# Launch scripts
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)


# scripts
install(PROGRAMS scripts/generate_wamv.py
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(
  vrx_gazebo
  PACKAGE_DIR src/vrx_gazebo
)
ament_package()
