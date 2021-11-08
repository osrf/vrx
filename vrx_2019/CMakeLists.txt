cmake_minimum_required(VERSION 2.8.3)
project(vrx_2019)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

find_package(catkin REQUIRED COMPONENTS
  vrx_gazebo
  wamv_gazebo
  xacro
)
catkin_package(
  CATKIN_DEPENDS vrx_gazebo wamv_gazebo xacro
)

set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

# Generate world files from xacro and install
xacro_add_files(
  worlds/dock.world.xacro
  worlds/example_course.world.xacro
  worlds/navigation_task.world.xacro
  worlds/ocean.world.xacro
  worlds/perception_task.world.xacro
  worlds/sandisland.world.xacro
  worlds/scan_and_dock.world.xacro
  worlds/stationkeeping_task.world.xacro
  worlds/wayfinding_task.world.xacro
  worlds/wind_test.world.xacro
  ${XACRO_INORDER} INSTALL DESTINATION worlds
)

# Install all the config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
