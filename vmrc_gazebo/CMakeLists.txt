cmake_minimum_required(VERSION 2.8.3)
project(vmrc_gazebo)

# We need erb to process the .world.erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

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

# Plugin for controlling symbols
add_library(symbol_controller src/symbol_controller.cc)
target_link_libraries(symbol_controller ${catkin_LIBRARIES})
add_dependencies(symbol_controller ${catkin_EXPORTED_TARGETS})
install(TARGETS symbol_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Dock base files that need to be processed with erb
set (dock_base_erb_files
  models/dock_2016_base/model.sdf.erb
  models/dock_2018_base/model.sdf.erb
)

# Dock files that need to be processed with erb
set (dock_erb_files
  models/dock_2016/model.sdf.erb
  models/dock_2018/model.sdf.erb
)

# Process the dock base erb files
foreach(_erb ${dock_base_erb_files})
  string(REGEX REPLACE ".sdf.erb" ".sdf" _model ${_erb})
  set(_model ${CMAKE_CURRENT_SOURCE_DIR}/${_model})
  add_custom_command(OUTPUT ${_model}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_model}
                     DEPENDS
                       ${CMAKE_CURRENT_SOURCE_DIR}/${_erb}
                       ${CMAKE_CURRENT_SOURCE_DIR}/dock_generator.erb
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND dock_base_files ${_model})
endforeach()
add_custom_target(dock_base_erb_generation ALL DEPENDS ${dock_base_files})

# Process the dock erb files
foreach(_erb ${dock_erb_files})
  string(REGEX REPLACE ".sdf.erb" ".sdf" _model ${_erb})
  set(_model ${CMAKE_CURRENT_SOURCE_DIR}/${_model})
  add_custom_command(OUTPUT ${_model}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_model}
                     DEPENDS
                       ${CMAKE_CURRENT_SOURCE_DIR}/${_erb}
                       ${CMAKE_CURRENT_SOURCE_DIR}/dock_generator.erb
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND dock_files ${_model})
endforeach()
add_custom_target(dock_erb_generation ALL
  DEPENDS
    ${dock_files}
    dock_base_erb_generation
)

# Generate world files from xacro and install
xacro_add_files(
  worlds/sandisland.world.xacro
  worlds/example_course.world.xacro
  INORDER INSTALL DESTINATION worlds
)

# Generate obstacle course
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_avoid_obstacles_buoys
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_avoid_obstacles_buoys --seed 1337 --a3 6 --a5 7 --a7 7 > ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf
)
add_custom_target(${PROJECT_NAME}_generate_obstacle_course_buoys ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf)

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
                    test/sandisland.cc)
  target_link_libraries(sandisland_test ${catkin_LIBRARIES})
endif()
