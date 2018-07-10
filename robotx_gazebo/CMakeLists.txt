cmake_minimum_required(VERSION 2.8.3)
project(robotx_gazebo)

find_package(catkin REQUIRED COMPONENTS
  wamv_gazebo
  xacro
  gazebo_dev
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_color
  CATKIN_DEPENDS wamv_gazebo xacro gazebo_dev std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

# Plugin for setting color of a gazebo visual through ROS
add_library(gazebo_ros_color src/gazebo_ros_color.cc)
target_link_libraries(gazebo_ros_color ${catkin_LIBRARIES})
add_dependencies(gazebo_ros_color ${catkin_EXPORTED_TARGETS})
install(TARGETS gazebo_ros_color
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Plugin for light buoy color sequence
add_library(light_buoy_controller src/light_buoy_controller.cc)
target_link_libraries(light_buoy_controller ${catkin_LIBRARIES})
add_dependencies(light_buoy_controller ${catkin_EXPORTED_TARGETS})
install(TARGETS light_buoy_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Generate world files from xacro and install
xacro_add_files(
  worlds/sandisland.world.xacro
  worlds/example_course.world.xacro
  INORDER INSTALL DESTINATION worlds
)

# Install all the config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(sandisland_test
                    test/sandisland.test
                    test/sandisland.cpp)
  target_link_libraries(sandisland_test ${catkin_LIBRARIES})
endif()
