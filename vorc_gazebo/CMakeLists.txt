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
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    gazebo_dev
    vrx_gazebo
    wamv_gazebo
    wave_gazebo
    xacro
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(gymkhana_scoring_plugin src/gymkhana_scoring_plugin.cc)
target_link_libraries(gymkhana_scoring_plugin ${catkin_LIBRARIES})

## Install ##

# Run xacro to generate world files
xacro_add_files(
  worlds/marina.world.xacro
  worlds/perception.world.xacro
  worlds/station_keeping.world.xacro
  worlds/wayfinding.world.xacro
  worlds/gymkhana.world.xacro
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

# Install all the executable nodes
catkin_install_python(
  PROGRAMS
    nodes/pinger_visualization.py
    nodes/set_pinger_position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS gymkhana_scoring_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
