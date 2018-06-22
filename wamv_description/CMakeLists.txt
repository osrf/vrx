cmake_minimum_required(VERSION 2.8.3)
project(wamv_description)
find_package(catkin REQUIRED COMPONENTS xacro)
catkin_package(CATKIN_DEPENDS xacro)
xacro_add_files(
  urdf/engine.xacro
  urdf/wamv_base.urdf.xacro
  urdf/wamv_two_engines.urdf.xacro
    INORDER INSTALL DESTINATION urdf
)
