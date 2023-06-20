cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(vrx_gz)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS graphics)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-fuel_tools8 REQUIRED)
find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
find_package(gz-msgs9 REQUIRED)
find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
find_package(gz-sensors7 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})
find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})
find_package(sdformat13 REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

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
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  Eigen3::Eigen
)
install(
  TARGETS Waves
  DESTINATION lib)

# Buoyancy
add_library(PolyhedraBuoyancyDrag SHARED
  src/PolyhedraBuoyancyDrag.cc
  src/PolyhedronVolume.cc
  src/ShapeVolume.cc
)
target_link_libraries(PolyhedraBuoyancyDrag PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  Waves
)
install(
  TARGETS PolyhedraBuoyancyDrag
  DESTINATION lib)


# Base scoring plugin
add_library(ScoringPlugin SHARED
  src/ScoringPlugin.cc
)
target_link_libraries(ScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
)
install(
  TARGETS ScoringPlugin
  DESTINATION lib)

# Stationkeeping scoring plugin
add_library(StationkeepingScoringPlugin SHARED
  src/StationkeepingScoringPlugin.cc
  src/WaypointMarkers.cc
)
target_link_libraries(StationkeepingScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  ScoringPlugin
)
install(
  TARGETS StationkeepingScoringPlugin
  DESTINATION lib)

# Wayfinding scoring plugin
add_library(WayfindingScoringPlugin SHARED
  src/WayfindingScoringPlugin.cc
  src/WaypointMarkers
)
target_link_libraries(WayfindingScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  ScoringPlugin
)
install(
  TARGETS WayfindingScoringPlugin
  DESTINATION lib)

# Acoustic perception scoring plugin
add_library(AcousticPerceptionScoringPlugin SHARED
  src/AcousticPerceptionScoringPlugin.cc
  src/WaypointMarkers
)
target_link_libraries(AcousticPerceptionScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  ScoringPlugin
)
install(
  TARGETS AcousticPerceptionScoringPlugin
  DESTINATION lib)

# AcousticTracking scoring plugin
add_library(AcousticTrackingScoringPlugin SHARED
  src/AcousticTrackingScoringPlugin.cc
  src/WaypointMarkers
)
target_link_libraries(AcousticTrackingScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  ScoringPlugin
)
install(
  TARGETS AcousticTrackingScoringPlugin
  DESTINATION lib)

# Plugins
list(APPEND VRX_GZ_PLUGINS
  AcousticPingerPlugin
  BallShooterPlugin
  LightBuoyPlugin
  NavigationScoringPlugin
  GymkhanaScoringPlugin
  PerceptionScoringPlugin
  PlacardPlugin
  PublisherPlugin
  ScanDockScoringPlugin
  SimpleHydrodynamics
  Surface
  USVWind
  WaveVisual
  WildlifeScoringPlugin
)

foreach(PLUGIN ${VRX_GZ_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    gz-sim${GZ_SIM_VER}::core
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    ScoringPlugin
    Waves
    Eigen3::Eigen
  )
endforeach()

install(
  TARGETS ${VRX_GZ_PLUGINS}
  DESTINATION lib)

ament_python_install_package(
  vrx_gz
  PACKAGE_DIR src/vrx_gz
)

#============================================================================
# Resources
install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
