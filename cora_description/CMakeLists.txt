cmake_minimum_required(VERSION 3.0.2)
project(cora_description)

# Set policy for CMake 3.1+, FindBoost.cmake dependency of gazebo uses quoted
# variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

find_package(catkin REQUIRED COMPONENTS
  wamv_gazebo
  xacro
)

catkin_package(
  CATKIN_DEPENDS xacro
)

xacro_add_files(
  urdf/cora.urdf.xacro
    INSTALL DESTINATION urdf
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
