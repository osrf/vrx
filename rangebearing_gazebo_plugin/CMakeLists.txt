cmake_minimum_required(VERSION 2.8.3)
project(rangebearing_gazebo_plugin)
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  message_generation
  xacro
)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS xacro
  )

# Plugins require c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories( #include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)


# Generate urdf files from xacro and install
xacro_add_files(
  urdf/rangebearing.urdf.xacro
  INORDER INSTALL DESTINATION urdf
)

# Install xacro files / macros
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

## Declare a C++ library
add_library(rangebearing_gazebo_plugin
  src/RangeBearing.cc
  )

target_link_libraries(rangebearing_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS rangebearing_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
