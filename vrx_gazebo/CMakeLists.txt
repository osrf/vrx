cmake_minimum_required(VERSION 3.5)
project(vrx_gazebo)

# We need erb to process the .world.erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# For Qt
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wamv_gazebo REQUIRED)
find_package(wave_gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

ament_export_dependencies(xacro)

########################
## Message generation ##
########################

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Task.msg
  msg/Contact.msg
  srv/ColorSequence.srv
  DEPENDENCIES builtin_interfaces std_msgs
#  SKIP_INSTALL
)

ament_export_dependencies(rosidl_default_runtime)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}") 

include_directories(include 
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  ${CMAKE_CURRENT_BINARY_DIR}/srv
  ${std_msgs_INCLUDE_DIRS}
)

add_subdirectory(msgs)

add_definitions(${QT_DEFINITIONS})

# Library for displaying waypoint markers
add_library(waypoint_markers src/waypoint_markers.cc)
target_link_libraries(waypoint_markers
    ${ament_cmake_LIBRARIES}
)
install(TARGETS waypoint_markers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for setting color of light buoy.
add_library(light_buoy_plugin src/light_buoy_plugin.cc)
target_link_libraries(light_buoy_plugin
  ${ament_cmake_LIBRARIES} 
  ${rclcpp_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  light_buoy_colors_msgs
)
install(TARGETS light_buoy_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for setting color and shape of a placard.
add_library(placard_plugin src/placard_plugin.cc)
target_link_libraries(placard_plugin
  ${ament_cmake_LIBRARIES}
  ${rclcpp_LIBRARIES}
  dock_placard_msgs
)
install(TARGETS placard_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for controlling the ball shooter.
add_library(ball_shooter_plugin src/ball_shooter_plugin.cc)
target_link_libraries(placard_plugin
  ${ament_cmake_LIBRARIES}
)
install(TARGETS ball_shooter_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for controlling the animal buoys .
add_library(follow_plugin src/follow_plugin.cc)
target_link_libraries(follow_plugin
  ${ament_cmake_LIBRARIES}
  waypoint_markers
)
install(TARGETS follow_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# A generic scoring plugin.
add_library(scoring_plugin src/scoring_plugin.cc)
target_link_libraries(scoring_plugin
  ${ament_cmake_LIBRARIES}
)
install(TARGETS scoring_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for scoring the navigation challenge task.
add_library(navigation_scoring_plugin src/navigation_scoring_plugin.cc)
target_link_libraries(navigation_scoring_plugin
  ${ament_cmake_LIBRARIES}
  scoring_plugin)
install(TARGETS navigation_scoring_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for scoring the wildlife challenge task.
add_library(wildlife_scoring_plugin src/wildlife_scoring_plugin.cc)
target_link_libraries(wildlife_scoring_plugin
  ${ament_cmake_LIBRARIES}
  ${geographic_msgs_LIBRARIES}
  scoring_plugin)
install(TARGETS wildlife_scoring_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for scoring the station keeping challenge task.
add_library(stationkeeping_scoring_plugin src/stationkeeping_scoring_plugin.cc)
target_link_libraries(stationkeeping_scoring_plugin
  ${ament_cmake_LIBRARIES}
  ${geographic_msgs_LIBRARIES}
  scoring_plugin
  waypoint_markers)
install(TARGETS stationkeeping_scoring_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for scoring the waypoint navigation challenge task.
add_library(wayfinding_scoring_plugin src/wayfinding_scoring_plugin.cc)
target_link_libraries(wayfinding_scoring_plugin
  ${ament_cmake_LIBRARIES}
  ${geographic_msgs_LIBRARIES}
  scoring_plugin
  waypoint_markers)
install(TARGETS wayfinding_scoring_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for scoring gymkhana task
add_library(gymkhana_scoring_plugin src/gymkhana_scoring_plugin.cc)
target_link_libraries(gymkhana_scoring_plugin
  ${ament_cmake_LIBRARIES}
  scoring_plugin
)

install(TARGETS gymkhana_scoring_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin for scoring the scan and dock task.
add_library(scan_dock_scoring_plugin src/scan_dock_scoring_plugin.cc)
target_link_libraries(scan_dock_scoring_plugin
  ${ament_cmake_LIBRARIES}
  ${rclcpp_LIBRARIES}
)
install(TARGETS scan_dock_scoring_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(${GAZEBO_MAJOR_VERSION} GREATER 7)
  # Plugin for Task Info GUI Overlay
  add_library(gui_task_widget SHARED ${headers_MOC}
         	src/gui_task_widget.cc
  )

  target_link_libraries(gui_task_widget 
    ${ament_cmake_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${QT_LIBRARIES}
  )
  qt5_use_modules(gui_task_widget Widgets)

  install(TARGETS gui_task_widget
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

# Plugin for scoring the perception task.
add_library(perception_scoring_plugin src/perception_scoring_plugin.cc)
target_link_libraries(perception_scoring_plugin
  ${ament_cmake_LIBRARIES}
  scoring_plugin)
install(TARGETS perception_scoring_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Connect targets with locally defined messages
get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(scoring_plugin ${PROJECT_NAME} ${typesupport_impl})
    rosidl_target_interfaces(navigation_scoring_plugin ${PROJECT_NAME} ${typesupport_impl})
    rosidl_target_interfaces(gymkhana_scoring_plugin ${PROJECT_NAME} ${typesupport_impl})
    rosidl_target_interfaces(scan_dock_scoring_plugin ${PROJECT_NAME} ${typesupport_impl})
    rosidl_target_interfaces(gui_task_widget ${PROJECT_NAME} ${typesupport_impl})  
  endforeach()
endforeach()


# Dock base files that need to be processed with erb
set(dock_base_erb_files
  models/dock_2016_base/model.sdf.erb
  models/dock_2018_base/model.sdf.erb
  models/dock_2022_base/model.sdf.erb
  models/dock_2016_base_dynamic/model.sdf.erb
  models/dock_2018_base_dynamic/model.sdf.erb
  models/dock_2022_base_dynamic/model.sdf.erb
)

# Dock files that need to be processed with erb
set(dock_erb_files
  models/dock_2016/model.sdf.erb
  models/dock_2018/model.sdf.erb
  models/dock_2022/model.sdf.erb
  models/dock_2016_dynamic/model.sdf.erb
  models/dock_2018_dynamic/model.sdf.erb
  models/dock_2022_dynamic/model.sdf.erb
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

# Generate obstacle course
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_avoid_obstacles_buoys.py
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_avoid_obstacles_buoys.py --seed 1337 --a3 6 --a5 7 --a7 7 > ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf
)
add_custom_target(${PROJECT_NAME}_generate_obstacle_course_buoys ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf)

# Install all the config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hh"
)

install(PROGRAMS scripts/spawn_wamv.bash
  DESTINATION lib/${PROJECT_NAME})

# Generate world files from xacro and install
if(ADD_XACRO)
  xacro_add_files(
    worlds/dock.world.xacro
    worlds/example_course.world.xacro
    worlds/example_course_2019.world.xacro
    worlds/gymkhana.world.xacro
    worlds/navigation_task.world.xacro
    worlds/ocean.world.xacro
    worlds/perception_task.world.xacro
    worlds/scan_and_dock.world.xacro
    worlds/scan_dock_deliver.world.xacro
    worlds/sandisland.world.xacro
    worlds/stationkeeping_task.world.xacro
    worlds/sydneyregatta.world.xacro
    worlds/wayfinding_task.world.xacro
    worlds/wildlife_task.world.xacro
    worlds/wind_test.world.xacro
    INSTALL DESTINATION worlds
  )
endif()

if(AMENT_ENABLE_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(gazebo_msgs REQUIRED)

  ament_add_gtest(sandisland_test test/sandisland.cc)

  target_link_libraries(sandisland_test 
    ${ament_cmake_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${gazebo_msgs_LIBRARIES}
  )
  
  install(TARGETS sandisland_test
    DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES test/sandisland.test
    DESTINATION share/${PROJECT_NAME}/test)
                  
endif()

# Python Scripts
set(PYTHON_SCRIPTS 
  nodes/pinger_visualization.py
  nodes/set_pinger_position.py
  nodes/twist2thrust.py
  nodes/key2thrust_angle.py
  scripts/generate_worlds.py
  scripts/generate_wamv.py
)

install(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
