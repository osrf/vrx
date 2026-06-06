cmake_minimum_required(VERSION 3.16)
project(gz_waves_provider_gerstner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# gz_waves's exported library links gz-sim PUBLIC (the Wavefield component), so
# gz-sim::core must be defined before we find_package(gz_waves). The gerstner
# provider itself only uses gz_waves's *headers* and does not link gz-sim; this
# is needed for the test, which links the core for CreateWaveSimulation.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_waves REQUIRED)          # IWaveField interface + WaveParameters
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# --------------------------------------------------------------------------
# Gerstner wave-field provider: a standalone gz-plugin discovered by the
# gz_waves core (CreateWaveSimulation) under the name gz-waves-provider-gerstner
# and instantiated through IWaveField. It depends only on the core headers
# (interface + params) plus gz-math — it never links a consumer or another
# provider (that would shadow the shared GzPluginHook symbol).
# --------------------------------------------------------------------------
add_library(gz-waves-provider-gerstner SHARED
  src/GerstnerWaveSimulation.cc
)
target_include_directories(gz-waves-provider-gerstner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # Core headers (IWaveField, WaveParameters) only — no link to the core lib:
  # the interface is abstract and the params are header-only.
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-waves-provider-gerstner
  PUBLIC  gz-math::gz-math
  PRIVATE gz-plugin::register)

install(
  TARGETS gz-waves-provider-gerstner
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # Gazebo (Allman) style, matching the upstream gz-sim source.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # This binary links exactly one provider (gerstner) and only ever loads that
  # one through CreateWaveSimulation, so the shared GzPluginHook is unambiguous.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(gerstner_test test/gerstner_test.cc)
  target_include_directories(gerstner_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${gz_waves_INCLUDE_DIRS})
  # Links the core (CreateWaveSimulation + Eval) and the gerstner provider
  # (the white-box GerstnerWaveSimulation accessors).
  target_link_libraries(gerstner_test
    gz_waves::gz_waves gz-waves-provider-gerstner)
  target_link_options(gerstner_test PRIVATE
    "LINKER:--unresolved-symbols=ignore-in-shared-libs")
endif()

ament_package()
