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

# Discover the Ogre Next headers. We must compile against the SAME Ogre Next the
# resolved gz-rendering-ogre2 uses (mixing patch versions silently swaps in slow
# Hlms/barrier paths). The previous approach parsed gz-rendering::ogre2's
# LOCATION, but that property is empty for imported/interface targets (it is
# NOTFOUND on the ROS gz vendor + Gazebo Jetty), so it wrongly fell back to a
# hardcoded /usr/include/OGRE-2.3. Instead, search for the OGRE-Next headers,
# seeded by the ogre2 target's own interface include dirs. PATH_SUFFIXES exclude
# a bare "OGRE" so we never pick up classic Ogre 1.x (/usr/include/OGRE,
# rviz_ogre_vendor), whose ABI-incompatible headers would break the build.
get_target_property(_ogre2_incs gz-rendering::ogre2 INTERFACE_INCLUDE_DIRECTORIES)

# A cached OGRE_NEXT_INCLUDE_DIR can go stale (e.g. the vendor was uninstalled);
# find_path would keep the old value even though the dir is gone. Drop it first
# so the search below re-runs whenever OgreRoot.h is no longer where we cached.
if(OGRE_NEXT_INCLUDE_DIR AND NOT EXISTS "${OGRE_NEXT_INCLUDE_DIR}/OgreRoot.h")
  unset(OGRE_NEXT_INCLUDE_DIR CACHE)
endif()

# Pass 1: vendor / target-hinted headers only. NO_DEFAULT_PATH skips the system
# dirs so a stray /usr/include/OGRE-2.x (a plain libogre-next-dev) can't shadow
# the gz vendor copy that gz-rendering-ogre2 was actually built against — mixing
# patch versions silently swaps in the slow Hlms/barrier paths.
find_path(OGRE_NEXT_INCLUDE_DIR
  NAMES OgreRoot.h
  HINTS ${_ogre2_incs}
  PATHS /opt/ros/$ENV{ROS_DISTRO}/opt/gz_ogre_next_vendor/include
  PATH_SUFFIXES OGRE-Next OGRE-2.4 OGRE-2.3
  NO_DEFAULT_PATH
)
# Pass 2: fall back to the system paths (a plain libogre-next-dev with no vendor
# present). Only runs when pass 1 left OGRE_NEXT_INCLUDE_DIR unset.
find_path(OGRE_NEXT_INCLUDE_DIR
  NAMES OgreRoot.h
  PATH_SUFFIXES OGRE-Next OGRE-2.4 OGRE-2.3
)
if(NOT OGRE_NEXT_INCLUDE_DIR)
  message(FATAL_ERROR
    "OGRE-Next headers (OgreRoot.h) not found. Install libogre-next-dev or the "
    "gz_ogre_next_vendor package.")
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

ament_package()
