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
find_package(Eigen3 REQUIRED)

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
    Eigen3::Eigen
  PRIVATE
    gz-common::gz-common
)

# Wave engines (gerstner, fft) live in their own packages as plain libraries,
# each paired with a system plugin (gz-sim-waves-<engine>-system) that links it
# directly. The core links no concrete engine.

# --------------------------------------------------------------------------
# Wave buoyancy system: a loadable plugin referenced by SDF filename. (The wave
# *source* systems moved to the provider packages, each linking its engine.)
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

# GZ_SIM_SYSTEM_PLUGIN_PATH hook so gz-sim finds the Waves / WaveBuoyancy
# system plugins (and the dlopen'd provider libs) by filename in install/lib.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_sim_vendor gz_math_vendor Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # Disable Google-style linters; this code follows the Gazebo (Allman) style
  # to match the upstream gz-sim source it will be merged into.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # The interface / factory registry / Eval / Wavefield component are exercised
  # by the provider packages' tests (gz_waves_provider_*), which link an engine
  # and register it. The core can't link an engine without a dependency cycle,
  # so it has no gtest of its own — only the lint checks above.
endif()

ament_package()
