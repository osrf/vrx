cmake_minimum_required(VERSION 2.8.3)
project(robotx_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  wamv_description
  xacro
)
catkin_package(
  CATKIN_DEPENDS wamv_description xacro
)

# Generate world files from xacro and install
file(GLOB ${PROJECT_NAME}_xacro_worlds RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} worlds/*.world.xacro)
unset(xacro_outputs)
foreach(f ${${PROJECT_NAME}_xacro_worlds})
  xacro_add_xacro_file(${f} INORDER)
  list(APPEND xacro_outputs ${XACRO_OUTPUT_FILE})
endforeach(f)
xacro_install(worlds ${xacro_outputs} DESTINATION worlds)
