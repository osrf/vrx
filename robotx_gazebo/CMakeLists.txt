cmake_minimum_required(VERSION 2.8.3)
project(robotx_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  hector_gazebo_plugins
  message_generation
  nav_msgs
  roscpp
  std_msgs
  wamv_description
)

## Gazebo
find_package(gazebo REQUIRED)

## Eigen
find_package(Eigen3 REQUIRED)

# Visual Studio enables c++11 support by default
if (NOT MSVC)
  set(UNFILTERED_FLAGS "-std=c++11")
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wall -Wextra")

########################
## Message generation ##
########################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  UsvDrive.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs nav_msgs
  LIBRARIES
)

###########
## Build ##
###########

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(buoyancy_gazebo_plugin
  src/buoyancy_gazebo_plugin.cc
 )
target_link_libraries(buoyancy_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS buoyancy_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_dynamics_plugin
  src/usv_gazebo_dynamics_plugin.cc
)
target_link_libraries(usv_gazebo_dynamics_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS usv_gazebo_dynamics_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_thrust_plugin
  src/usv_gazebo_thrust_plugin.cc
)
target_link_libraries(usv_gazebo_thrust_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS usv_gazebo_thrust_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_wind_plugin
  src/usv_gazebo_wind_plugin.cc
)
target_link_libraries(usv_gazebo_wind_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS usv_gazebo_wind_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#############
## Install ##
#############

# Install all the config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Configure the setup script
#catkin_add_env_hooks(99_robotx_gazebo_setup
#  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
#  SHELLS sh)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robotx_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
