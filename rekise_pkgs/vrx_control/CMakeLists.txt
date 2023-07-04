cmake_minimum_required(VERSION 3.8)
project(vrx_control)
find_package(ament_cmake REQUIRED)

# Launch scripts
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
  
ament_package()
