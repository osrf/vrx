cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(vrx_gazebo)
find_package(ament_cmake REQUIRED)

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# Models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_package()
