cmake_minimum_required(VERSION 3.5)
project(wamv_description)
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(ADD_XACRO)
  xacro_add_files(
    urdf/wamv_base.urdf.xacro
      INSTALL DESTINATION urdf
  )
endif()

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

ament_package()
