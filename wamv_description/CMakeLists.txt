cmake_minimum_required(VERSION 2.8.3)
project(wamv_description)
find_package(catkin REQUIRED COMPONENTS xacro)
catkin_package(CATKIN_DEPENDS xacro)
xacro_add_files(
  urdf/wamv_base.urdf.xacro
    INORDER INSTALL DESTINATION urdf
)

install(DIRECTORY mesh/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
