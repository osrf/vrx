cmake_minimum_required(VERSION 3.16)
project(gz_waves)

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
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# --------------------------------------------------------------------------
# Core wave library: the IWaveField interface, the WavesSystemBase plumbing, the
# token -> factory registry (CreateWaveSimulation / RegisterWaveEngineFactory),
# and the Wavefield component. It links NO concrete engine — engines are plain
# libraries in their own packages, linked by the per-engine system plugins and
# the water visual — so the core stays backend-agnostic.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/WaveSimulation.cc
  src/components.cc
  src/Eval.cc
  src/systems/WavesSystemBase.cc
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
  PRIVATE
    gz-common::gz-common
)

# The core ships only the wave-field contract. Everything else lives in its own
# package: the engines (gz_waves_provider_{gerstner,fft}) as plain libraries each
# paired with a gz-sim-waves-<engine>-system source plugin, and the consumers of
# the field — buoyancy (gz_waves_buoyancy) and the water visual
# (gz_waves_rendering). None is built here.

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

# GZ_SIM_SYSTEM_PLUGIN_PATH hook so gz-sim finds the Waves / WaveBuoyancy
# system plugins (and the dlopen'd provider libs) by filename in install/lib.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_sim_vendor gz_math_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # Disable Google-style linters; this code follows the Gazebo (Allman) style
  # to match the upstream gz-sim source it will be merged into.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # The core can't link a *real* engine (dependency cycle), but a test can supply
  # a stub IWaveField + stub WavesSystemBase and drive the whole contract — Eval,
  # the factory registry, Wavefield serialization, the sea-state helpers, and the
  # source-system plumbing (Configure / PreUpdate / Reset / set_parameters) —
  # through a real EntityComponentManager. No provider dependency, no cycle.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(wave_core_test test/wave_core_test.cc)
  target_link_libraries(wave_core_test ${PROJECT_NAME} gz-sim::core)
endif()

ament_package()
