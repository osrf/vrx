cmake_minimum_required(VERSION 3.0.2)
project(vorc_gazebo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Set policy for CMake 3.1+, FindBoost.cmake dependency of gazebo uses quoted
# variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  vrx_gazebo
  wamv_gazebo
  wave_gazebo
  xacro
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_dev
    vrx_gazebo
    wamv_gazebo
    wave_gazebo
    xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Install ##

# Run xacro to generate world files
xacro_add_files(
  worlds/marina.world.xacro
  INSTALL DESTINATION worlds
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

