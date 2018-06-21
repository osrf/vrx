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

set(${PROJECT_NAME}_xacro_urdf
  urdf/wamv_gazebo_sensors.urdf.xacro
  urdf/wamv_gazebo.urdf.xacro
)
unset(xacro_outputs)
foreach(f ${${PROJECT_NAME}_xacro_urdf})
  xacro_add_xacro_file(${f} INORDER)
  list(APPEND xacro_outputs ${XACRO_OUTPUT_FILE})
endforeach(f)
xacro_install(urdf ${xacro_outputs} DESTINATION urdf)
