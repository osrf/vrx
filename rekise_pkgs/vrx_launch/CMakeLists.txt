cmake_minimum_required(VERSION 3.8)
project(vrx_launch)
find_package(ament_cmake REQUIRED)

# Launch scripts
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)


ament_package()