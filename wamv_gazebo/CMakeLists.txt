cmake_minimum_required(VERSION 3.5)
project(wamv_gazebo)


find_package(ament_cmake REQUIRED)
find_package(wamv_description REQUIRED)
find_package(usv_gazebo_plugins REQUIRED)
find_package(xacro REQUIRED)

# Generate urdf files from xacro and install
if(ADD_XACRO)
  xacro_add_files(
    urdf/wamv_gazebo.urdf.xacro
    INSTALL DESTINATION urdf
  )
endif()

# Install meshes and textures
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

# Install xacro files / macros
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
