cmake_minimum_required(VERSION 3.16)
project(gz_waves_provider_gerstner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# gz_waves's exported library links gz-sim PUBLIC (the Wavefield component), so
# gz-sim::core must be resolvable before find_package(gz_waves). The core also
# provides WavesSystemBase, the base for the gerstner system plugin below.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_waves REQUIRED)          # IWaveField + WaveParameters + base
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)           # gz::math vectors + GZ_PI
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)         # gzerr/gzwarn console

# --------------------------------------------------------------------------
# Gerstner wave-field engine (libgz-waves-provider-gerstner): a plain analytic
# IWaveField implementation. It is LINKED directly by the gerstner system
# plugin (server) and the water visual (GUI), not discovered by a runtime
# loader, so it carries no GZ_ADD_PLUGIN and is exported as a normal target.
# MakeGerstnerWaveField() is the factory consumers register under "gerstner".
# --------------------------------------------------------------------------
add_library(gz-waves-provider-gerstner SHARED
  src/GerstnerWaveSimulation.cc
)
target_include_directories(gz-waves-provider-gerstner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(gz-waves-provider-gerstner PRIVATE
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-waves-provider-gerstner
  PUBLIC gz-math::gz-math
  PRIVATE gz-common::gz-common)

# --------------------------------------------------------------------------
# Gerstner wave source system (gz-sim-waves-gerstner-system): a WavesSystemBase
# subclass that builds the Gerstner engine. This is the gz-plugin SDF loads by
# filename; it links the engine directly, so no runtime engine loader is used.
# --------------------------------------------------------------------------
add_library(gz-sim-waves-gerstner-system SHARED
  src/GerstnerWavesSystem.cc
)
target_include_directories(gz-sim-waves-gerstner-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-sim-waves-gerstner-system
  PRIVATE
    gz_waves::gz_waves
    gz-waves-provider-gerstner
    gz-sim::core
    gz-plugin::register)

# --------------------------------------------------------------------------
# Gerstner GUI registrar (gz-sim-waves-gerstner-gui): loaded in the GUI process
# alongside WaterVisual. It registers the "gerstner" factory so the visual can
# rebuild its own thread-private engine from the replicated recipe via
# CreateWaveSimulation. This is what lets gz_waves_rendering stay
# provider-agnostic (core-only): the engine dependency lives here, the GUI-side
# mirror of gz-sim-waves-gerstner-system.
# --------------------------------------------------------------------------
add_library(gz-sim-waves-gerstner-gui SHARED
  src/GerstnerWavesGui.cc
)
target_include_directories(gz-sim-waves-gerstner-gui PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-sim-waves-gerstner-gui
  PRIVATE
    gz_waves::gz_waves
    gz-waves-provider-gerstner
    gz-sim::core
    gz-plugin::register)

install(
  TARGETS gz-waves-provider-gerstner
  EXPORT export_gz_waves_provider_gerstner
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS gz-sim-waves-gerstner-system gz-sim-waves-gerstner-gui
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_gz_waves_provider_gerstner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_math_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # Gazebo (Allman) style, matching the upstream gz-sim source.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Links the engine directly and registers its "gerstner" factory in-binary
  # (see kGerstnerRegistered in the test), so the CreateWaveSimulation tests
  # resolve via the registry — no dlopen, no GzPluginHook.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(gerstner_test test/gerstner_test.cc)
  target_include_directories(gerstner_test PRIVATE
    ${gz_waves_INCLUDE_DIRS})
  target_link_libraries(gerstner_test
    gz_waves::gz_waves gz-waves-provider-gerstner)
endif()

ament_package()
