cmake_minimum_required(VERSION 3.16)
project(vrx_usv_dynamics)

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
find_package(gz_waves REQUIRED)   # Wavefield component + Eval free functions
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Surface: per-point circle-segment buoyancy. A wave-field CONSUMER - it reads
# components::Wavefield off the world entity and queries it through the Eval
# free functions, never including an engine header (gz_waves invariant N3).
#
# INTERIM TARGET. This is the only thing in the package that depends on
# gz_waves, and it duplicates what gz_waves_buoyancy's WaveBuoyancy will do
# once it lands. Retiring it should be a deletion, not a refactor: drop this
# target, its two source files, the install entry below, and the
# find_package(gz_waves) line above. See README.md.
# ---------------------------------------------------------------------------
add_library(vrx-surface-system SHARED src/Surface.cc)
target_include_directories(vrx-surface-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(vrx-surface-system
  PRIVATE
    gz_waves::gz_waves        # components::Wavefield + waves::SurfaceElevation
    gz-sim::core
    gz-math::gz-math
    gz-common::gz-common
    gz-plugin::register
)
install(
  TARGETS vrx-surface-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------------------------
# SimpleHydrodynamics: Fossen added-mass, Coriolis and linear/quadratic drag,
# applied from the link's own velocity and acceleration.
#
# Deliberately does NOT link gz_waves - it has no wave-field dependency, so it
# survives whatever happens to the buoyancy story above and could be moved or
# upstreamed independently.
# ---------------------------------------------------------------------------
add_library(vrx-simple-hydrodynamics-system SHARED src/SimpleHydrodynamics.cc)
target_include_directories(vrx-simple-hydrodynamics-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(vrx-simple-hydrodynamics-system
  PRIVATE
    gz-sim::core
    gz-math::gz-math
    gz-common::gz-common
    gz-plugin::register
    Eigen3::Eigen
)
install(
  TARGETS vrx-simple-hydrodynamics-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# GZ_SIM_SYSTEM_PLUGIN_PATH so gz-sim resolves filename="vrx-surface-system"
# and filename="vrx-simple-hydrodynamics-system" from model SDF.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Ported files keep their upstream OSRF copyright header.
  set(ament_cmake_copyright_FOUND TRUE)
  # Gazebo (Allman) house style, as in gz_waves.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
