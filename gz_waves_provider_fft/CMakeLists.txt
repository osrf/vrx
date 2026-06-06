cmake_minimum_required(VERSION 3.16)
project(gz_waves_provider_fft)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# gz_waves's exported library links gz-sim PUBLIC, so gz-sim::core must be
# defined before find_package(gz_waves) (needed only for the test, which links
# the core for CreateWaveSimulation — the provider links headers only).
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_waves REQUIRED)          # IWaveField interface + WaveParameters
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
# Apache-2.0 Horvath spectrum library; provides Eigen transitively.
find_package(encino_waves REQUIRED)

# --------------------------------------------------------------------------
# FFT wave-field provider: a standalone gz-plugin discovered by the gz_waves
# core (CreateWaveSimulation) under the name gz-waves-provider-fft and
# instantiated through IWaveField. Links the core headers + EncinoWaves; never
# the core library or another provider.
# --------------------------------------------------------------------------
add_library(gz-waves-provider-fft SHARED
  src/FFTWaveSimulation.cc
)
target_include_directories(gz-waves-provider-fft PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-waves-provider-fft
  PUBLIC  gz-math::gz-math
  PRIVATE gz-plugin::register encino_waves::encino_waves)

install(
  TARGETS gz-waves-provider-fft
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # FFTWaveSimulation.hh exposes Eigen in its public interface (grid accessors
  # return Eigen::MatrixXd), so the test needs Eigen directly.
  find_package(Eigen3 REQUIRED)

  # Links exactly one provider (fft) and only ever loads that one through
  # CreateWaveSimulation, so the shared GzPluginHook is unambiguous.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(fft_test test/fft_test.cc)
  target_include_directories(fft_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${gz_waves_INCLUDE_DIRS})
  target_link_libraries(fft_test
    gz_waves::gz_waves gz-waves-provider-fft Eigen3::Eigen)
  target_link_options(fft_test PRIVATE
    "LINKER:--unresolved-symbols=ignore-in-shared-libs")
endif()

ament_package()
