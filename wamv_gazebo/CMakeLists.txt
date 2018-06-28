cmake_minimum_required(VERSION 2.8.3)
project(wamv_gazebo)
find_package(catkin REQUIRED COMPONENTS
  wamv_description
  usv_gazebo_plugins
  xacro
)
catkin_package(
  CATKIN_DEPENDS wamv_description usv_gazebo_plugins xacro
)

# Generate urdf files from xacro and install
xacro_add_files(
  urdf/wamv_gazebo.urdf.xacro
  urdf/wamv_gazebo_sensors.urdf.xacro
  INORDER INSTALL DESTINATION urdf
)
