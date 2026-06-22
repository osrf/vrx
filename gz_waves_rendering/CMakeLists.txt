cmake_minimum_required(VERSION 3.16)
project(gz_waves_rendering)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_waves REQUIRED)   # IWaveField + WaveField2D + Wavefield component
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED COMPONENTS ogre2)
# NOTE: this package is provider-agnostic — it does NOT depend on the wave
# engines. WaterVisual rebuilds its private engine through the gz_waves core
# registry (CreateWaveSimulation); the GUI-side factory registration lives in
# the per-engine GUI plugins (gz-sim-waves-<engine>-gui), loaded by the world.

# Discover Ogre Next headers. Different Gazebo installs sit on different Ogre
# Next patch versions (ROS gz_ogre_next_vendor 2.3.3 vs Ubuntu libogre-next-dev
# 2.3.1). Compiling against one but loading the other silently swaps in slow
# Hlms/barrier code paths (no crash, but RTF tanks), so probe which Ogre Next
# the *resolved* gz-rendering-ogre2 links against and match our header path.
get_target_property(_gz_rend_ogre2_lib gz-rendering::ogre2 LOCATION)
if(_gz_rend_ogre2_lib MATCHES "/opt/ros/([^/]+)/")
  set(_ros_distro "${CMAKE_MATCH_1}")
  set(OGRE_NEXT_INCLUDE_DIR
    "/opt/ros/${_ros_distro}/opt/gz_ogre_next_vendor/include/OGRE-Next"
    CACHE PATH "Ogre Next include dir matched to gz-rendering-ogre2 distro")
else()
  # Non-ROS-distro install (e.g. local jetty_ws build) → system Ogre Next.
  set(OGRE_NEXT_INCLUDE_DIR "/usr/include/OGRE-2.3"
    CACHE PATH "Ogre Next include dir from libogre-next-dev")
endif()
if(NOT EXISTS "${OGRE_NEXT_INCLUDE_DIR}/OgreRoot.h")
  message(FATAL_ERROR
    "OgreRoot.h not found under ${OGRE_NEXT_INCLUDE_DIR} "
    "(probed from gz-rendering::ogre2 = ${_gz_rend_ogre2_lib}). "
    "Install libogre-next-dev or gz_ogre_next_vendor as appropriate.")
endif()
message(STATUS "Ogre Next headers: ${OGRE_NEXT_INCLUDE_DIR}")

# The Ogre Next dynamic heightmap-texture helper lives in a SEPARATE shared
# library. Linking it directly into WaterVisual would force the plugin's
# DT_NEEDED to include libgz-rendering-ogre2.so, which interferes with
# gz-rendering's own engine-plugin loader (same SONAME already in process when
# gz-rendering tries to dlopen its engine plugin → the engine never initialises
# and the GUI shows a default-material plane). WaterVisual loads this bridge on
# demand via dlopen, so its symbols never enter the WaterVisual plugin's link
# map (mirrors the asv_wave_sim / gz_fft_waves rendering-ogre2 split).
add_library(waves-ogre2-bridge SHARED
  src/systems/Ogre2HeightMapBridge.cc
)
target_link_libraries(waves-ogre2-bridge
  PRIVATE
    gz-rendering::gz-rendering
    gz-rendering::ogre2
    gz-common::gz-common
)
# SYSTEM so Ogre Next's own headers (extra ';', unused params, etc.) don't
# drown our -Wall -Wextra -Wpedantic output — they are upstream Ogre, not ours.
target_include_directories(waves-ogre2-bridge SYSTEM PRIVATE
  ${OGRE_NEXT_INCLUDE_DIR}
  # HlmsPbs headers transitively include Hlms/Common/OgreHlmsBufferManager.h
  # by unqualified name — that subdirectory must also be on the path.
  ${OGRE_NEXT_INCLUDE_DIR}/Hlms/Common
  ${OGRE_NEXT_INCLUDE_DIR}/Hlms/Pbs
)
# Ogre can't infer debug/release from our flags and emits a #pragma message in
# every TU; silence it (RelWithDebInfo is not an Ogre debug build).
target_compile_definitions(waves-ogre2-bridge PRIVATE OGRE_IGNORE_UNKNOWN_DEBUG)
install(
  TARGETS waves-ogre2-bridge
  LIBRARY DESTINATION lib
)

# WaterVisual: a gz-sim rendering system. Reads the wave field from the
# Wavefield component through IWaveField::Field() (gz_waves core) and uploads
# it as a displacement heightmap. Links only the engine-agnostic gz-rendering
# core; the Ogre2 bridge is loaded via dlopen (CMAKE_DL_LIBS).
add_library(gz-sim-water-visual-system SHARED
  src/systems/WaterVisual.cc
  src/systems/HeightMapTexture.cc
)
target_include_directories(gz-sim-water-visual-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/systems>
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-sim-water-visual-system
  PRIVATE
    gz_waves::gz_waves        # IWaveField + Field() + Wavefield component +
                              # CreateWaveSimulation (engine-agnostic registry)
    gz-sim::core
    gz-math::gz-math
    gz-plugin::register
    gz-rendering::gz-rendering
    gz-common::gz-common
    ${CMAKE_DL_LIBS}
)
install(
  TARGETS gz-sim-water-visual-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Water-surface model + shaders + meshes (model://water_surface).
install(
  DIRECTORY share/models
  DESTINATION share/${PROJECT_NAME}
)

# GZ_SIM_RESOURCE_PATH (water_surface model) + GZ_SIM_SYSTEM_PLUGIN_PATH hooks.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

if(BUILD_TESTING)
  # Unit test for the pure column->row reflow helper on the heightmap upload
  # path. Header-only and Ogre-free, so it links nothing but gtest.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(grid_reflow_test test/grid_reflow_test.cc)
  target_include_directories(grid_reflow_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/systems)
endif()

ament_package()
