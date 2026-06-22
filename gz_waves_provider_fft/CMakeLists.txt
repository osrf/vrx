cmake_minimum_required(VERSION 3.16)
project(gz_waves_provider_fft)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# gz_waves's exported library links gz-sim PUBLIC, so gz-sim::core must be
# resolvable before find_package(gz_waves). The core also provides
# WavesSystemBase, the base for the fft system plugin below.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_waves REQUIRED)          # IWaveField + WaveParameters + base
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)           # gz::math::Vector3d in the interface
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)         # gzerr/gzmsg console
# FFTWaveSimulation's public header returns Eigen::MatrixXd grids, so Eigen is
# a direct dependency.
find_package(Eigen3 REQUIRED)
# Apache-2.0 Horvath spectrum library — REQUIRED, consumed as a regular system
# package (installed from HonuRobotics/encinowaves, not vendored in-tree). The
# fft engine is the EncinoWaves spectral synthesizer (TMA/JONSWAP/PM spectra +
# directional spreading + dispersion), selected via
# <spectrum>/<spreading>/<dispersion>. Its CMake config transitively pulls in
# Eigen3 / TBB / Imath.
find_package(EncinoWaves REQUIRED)

# --------------------------------------------------------------------------
# FFT wave-field engine (libgz-waves-provider-fft): a plain IWaveField
# implementation backed by the EncinoWaves spectral library.
# It is LINKED directly by the fft system plugin (server) and the water visual
# (GUI), not discovered by a runtime loader, so it carries no GZ_ADD_PLUGIN and
# is exported as a normal library target. MakeFFTWaveField() is the factory the
# consumers register under the "fft" token.
# --------------------------------------------------------------------------
add_library(gz-waves-provider-fft SHARED
  src/FFTWaveSimulation.cc
)
target_include_directories(gz-waves-provider-fft PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(gz-waves-provider-fft PRIVATE
  ${gz_waves_INCLUDE_DIRS})
# Eigen is PUBLIC: the engine's public header returns Eigen::MatrixXd grids, so
# anything that includes it (the system plugin, the water visual) needs Eigen.
target_link_libraries(gz-waves-provider-fft
  PUBLIC  Eigen3::Eigen gz-math::gz-math
  PRIVATE EncinoWaves::EncinoWaves gz-common::gz-common)

# --------------------------------------------------------------------------
# FFT wave source system (gz-sim-waves-fft-system): a WavesSystemBase subclass
# that builds the FFT engine. THIS is the gz-plugin SDF loads by filename; it
# links the engine directly, so there is no runtime engine loader on the server.
# --------------------------------------------------------------------------
add_library(gz-sim-waves-fft-system SHARED
  src/FftWavesSystem.cc
)
target_include_directories(gz-sim-waves-fft-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-sim-waves-fft-system
  PRIVATE
    gz_waves::gz_waves
    gz-waves-provider-fft
    gz-sim::core
    gz-plugin::register)

# --------------------------------------------------------------------------
# FFT GUI registrar (gz-sim-waves-fft-gui): loaded in the GUI process alongside
# WaterVisual. It registers the "fft" factory so the visual can rebuild its own
# thread-private engine from the replicated recipe via CreateWaveSimulation.
# This is what lets gz_waves_rendering stay provider-agnostic (core-only): the
# engine dependency lives here, the GUI-side mirror of gz-sim-waves-fft-system.
# --------------------------------------------------------------------------
add_library(gz-sim-waves-fft-gui SHARED
  src/FftWavesGui.cc
)
target_include_directories(gz-sim-waves-fft-gui PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-sim-waves-fft-gui
  PRIVATE
    gz_waves::gz_waves
    gz-waves-provider-fft
    gz-sim::core
    gz-plugin::register)

install(
  TARGETS gz-waves-provider-fft
  EXPORT export_gz_waves_provider_fft
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS gz-sim-waves-fft-system gz-sim-waves-fft-gui
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_gz_waves_provider_fft HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_math_vendor Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Links the engine directly and registers its "fft" factory in-binary (see
  # kFftRegistered in the test), so the CreateWaveSimulation tests resolve via
  # the registry — no dlopen, no GzPluginHook.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(fft_test test/fft_test.cc)
  target_include_directories(fft_test PRIVATE
    ${gz_waves_INCLUDE_DIRS})
  target_link_libraries(fft_test
    gz_waves::gz_waves gz-waves-provider-fft)
endif()

ament_package()
