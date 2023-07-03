cmake_minimum_required(VERSION 3.8)
project(vrx_localization)
find_package(ament_cmake REQUIRED)

# Launch scripts
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Launch params
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params)

ament_package()