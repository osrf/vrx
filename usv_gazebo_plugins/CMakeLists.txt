cmake_minimum_required(VERSION 3.5)
project(usv_gazebo_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(usv_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wave_gazebo_plugins REQUIRED)


set(INCLUDE_DIRS include 
  ${sensor_msgs_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS} 
  ${std_msgs_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${wave_gazebo_plugins_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${usv_msgs_INCLUDE_DIRS}
)

include_directories(${INCLUDE_DIRS})

link_directories(
  ${ament_cmake_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS}
  ${gazebo_dev_LIBRARY_DIRS}
  ${geometry_msgs_LIBRARY_DIRS}
  ${std_msgs_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS}
  ${wave_gazebo_plugins_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(buoyancy_gazebo_plugin SHARED
  src/buoyancy_gazebo_plugin.cc
  src/shape_volume.cc
  src/polyhedron_volume.cc
)
target_link_libraries(buoyancy_gazebo_plugin
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${wave_gazebo_plugins_LIBRARIES}
)
install(TARGETS buoyancy_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_dynamics_plugin SHARED
  src/usv_gazebo_dynamics_plugin.cc
)
target_link_libraries(usv_gazebo_dynamics_plugin
  ${Eigen_LIBRARIES}
  ${gazebo_dev_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${wave_gazebo_plugins_LIBRARIES}
)
install(TARGETS usv_gazebo_dynamics_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_thrust_plugin SHARED
  src/usv_gazebo_thrust_plugin.cc
)
target_link_libraries(usv_gazebo_thrust_plugin
  ${ament_cmake_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${gazebo_dev_LIBRARIES}
  ${rclcpp_LIBRARIES} 
)
install(TARGETS usv_gazebo_thrust_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_wind_plugin SHARED
  src/usv_gazebo_wind_plugin.cc
)
target_link_libraries(usv_gazebo_wind_plugin
  ${ament_cmake_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${rclcpp_LIBRARIES}
)
install(TARGETS usv_gazebo_wind_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_acoustic_pinger_plugin SHARED
  src/acoustic_pinger_plugin.cc
)

target_link_libraries(usv_gazebo_acoustic_pinger_plugin
  ${ament_cmake_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${rclcpp_LIBRARIES}
)
install(TARGETS usv_gazebo_acoustic_pinger_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_target_dependencies(usv_gazebo_acoustic_pinger_plugin
  rclcpp
  usv_msgs
  geometry_msgs
)

ament_export_libraries(buoyancy_gazebo_plugin)
ament_export_libraries(usv_gazebo_dynamics_plugin)
ament_export_libraries(usv_gazebo_thrust_plugin)
ament_export_libraries(usv_gazebo_wind_plugin)
ament_export_libraries(usv_gazebo_acoustic_pinger_plugin)

# Generate demo world files from xacro and install
xacro_add_files(
  worlds/buoyancy_plugin_demo.world.xacro
  INSTALL DESTINATION worlds
)
install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # buoyancy plugin test
  ament_add_gtest(buoyancy_plugin_test test/buoyancy_test.cc)
  target_link_libraries(buoyancy_plugin_test buoyancy_gazebo_plugin)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include)

ament_package()
