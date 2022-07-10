cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(vrx_ign)

include_directories("/opt/ros/galactic/include")
include_directories("~/vrx_ros2_ws/install/vrx_ros/include")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED COMPONENTS graphics)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-fuel_tools7 REQUIRED)
find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-rendering6 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})
find_package(sdformat12 REQUIRED)
find_package(ignition-physics4 REQUIRED)
set(IGN_PHYSICS_VER ${ignition-physics4_VERSION_MAJOR})

find_package(std_msgs REQUIRED)
#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

#============================================================================

# Waves
add_library(Waves SHARED
  src/Wavefield.cc
)
target_link_libraries(Waves PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-math${IGN_MATH_VER}
)
install(
  TARGETS Waves
  DESTINATION lib)

# Plugins
list(APPEND VRX_IGN_PLUGINS
  SimpleHydrodynamics
  Surface
  WaveVisual
  DummyPlugin
  ScoringPlugin
)

foreach(PLUGIN ${VRX_IGN_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER}
    Waves
  )
endforeach()

ament_python_install_package(
  vrx_ign
  PACKAGE_DIR src/vrx_ign
)

#============================================================================
# Resources
install(DIRECTORY
  worlds
  models
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
