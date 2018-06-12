cmake_minimum_required(VERSION 2.8.3)
project(wamv_description)
find_package(catkin REQUIRED COMPONENTS xacro)
catkin_package(CATKIN_DEPENDS xacro)
file(GLOB ${PROJECT_NAME}_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/wamv_base.urdf.xacro)
xacro_add_files(${${PROJECT_NAME}_xacro_files} INORDER INSTALL DESTINATION urdf)

