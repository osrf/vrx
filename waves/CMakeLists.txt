cmake_minimum_required(VERSION 3.16)
project(waves)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# --------------------------------------------------------------------------
# Core wave-simulation library: backend interface, factory, Gerstner impl.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/WaveSimulation.cc
  src/GerstnerWaveSimulation.cc
  src/FFTWaveSimulation.cc
  src/components.cc
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    gz-sim::core
    gz-math::gz-math
)

# --------------------------------------------------------------------------
# Gazebo systems. Each is a separate loadable plugin so SDF references them
# by filename (libgz-sim-waves-system, etc.) — matches upstream naming.
# --------------------------------------------------------------------------
function(add_waves_system NAME SOURCE)
  add_library(${NAME} SHARED ${SOURCE})
  target_link_libraries(${NAME}
    PRIVATE
      ${PROJECT_NAME}
      gz-sim::core
      gz-math::gz-math
      gz-plugin::register
  )
  target_include_directories(${NAME}
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  install(
    TARGETS ${NAME}
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

add_waves_system(gz-sim-waves-system        src/systems/Waves.cc)
add_waves_system(gz-sim-water-visual-system src/systems/WaterVisual.cc)
# Thin dlopen-based wrapper for the Ogre2 heightmap bridge. Compiles
# without any Ogre2 headers, so WaterVisual's plugin .so stays Ogre2-free.
target_sources(gz-sim-water-visual-system
  PRIVATE
    src/systems/HeightMapTexture.cc
)

find_package(gz-rendering REQUIRED COMPONENTS ogre2)
find_package(gz_ogre_next_vendor REQUIRED)

# The Ogre Next dynamic heightmap-texture helper for the FFT visual path
# lives in a SEPARATE shared library. Linking it directly into WaterVisual
# would force the plugin's DT_NEEDED to include libgz-rendering-ogre2.so,
# which interferes with gz-rendering's own engine-plugin loader (same SONAME
# already in process when gz-rendering tries to dlopen its engine plugin →
# the engine never initialises and the GUI shows a default-material plane).
# WaterVisual loads this bridge on demand via dlopen, so its symbols never
# enter the WaterVisual plugin's link map. This mirrors the asv_wave_sim /
# gz_fft_waves split (their "rendering-ogre2" component is isolated from
# the visual plugin in exactly the same way).
add_library(waves-ogre2-bridge SHARED
  src/systems/Ogre2HeightMapBridge.cc
)
target_link_libraries(waves-ogre2-bridge
  PRIVATE
    gz-rendering::gz-rendering
    gz-rendering::ogre2
    gz-common::gz-common
)
target_include_directories(waves-ogre2-bridge PRIVATE
  /opt/ros/rolling/opt/gz_ogre_next_vendor/include/OGRE-Next
  # HlmsPbs headers transitively include Hlms/Common/OgreHlmsBufferManager.h
  # by unqualified name — that subdirectory must also be on the path.
  /opt/ros/rolling/opt/gz_ogre_next_vendor/include/OGRE-Next/Hlms/Common
  /opt/ros/rolling/opt/gz_ogre_next_vendor/include/OGRE-Next/Hlms/Pbs
)
install(
  TARGETS waves-ogre2-bridge
  LIBRARY DESTINATION lib
)

# WaterVisual itself links only the engine-agnostic gz-rendering core.
target_link_libraries(gz-sim-water-visual-system
  PRIVATE
    gz-rendering::gz-rendering
    gz-common::gz-common
    ${CMAKE_DL_LIBS}
)
add_waves_system(gz-sim-wave-buoyancy-system src/systems/WaveBuoyancy.cc)

# --------------------------------------------------------------------------
# Headers and shared assets (shaders, textures, mesh).
# --------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY share/models
  DESTINATION share/${PROJECT_NAME}
)

# GZ_SIM_RESOURCE_PATH hook so worlds can `<include><uri>model://water_surface`.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gz_sim_vendor gz_math_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # Disable Google-style linters; this code follows the Gazebo (Allman) style
  # to match the upstream gz-sim source it will be merged into.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(eval_test test/eval_test.cc)
  target_link_libraries(eval_test ${PROJECT_NAME})

  ament_add_gtest(fft_test test/fft_test.cc)
  target_link_libraries(fft_test ${PROJECT_NAME})

  # Some Gazebo Jetty source builds ship a `libgz-transport.so` whose
  # DT_NEEDED libzenohc.so isn't accompanied by a DT_RUNPATH/RPATH. ld then
  # can't walk the transitive shared-lib graph to find Zenoh's symbols
  # while linking our tests, even though libzenohc.so sits in /usr/lib and
  # resolves cleanly at runtime. Tolerate that here so the tests build
  # against custom Gazebo workspaces without further env setup.
  foreach(_t IN ITEMS eval_test fft_test)
    target_link_options(${_t} PRIVATE
      "LINKER:--unresolved-symbols=ignore-in-shared-libs")
  endforeach()
endif()

ament_package()
