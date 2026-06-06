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
# FFTWaveSimulation's public header returns Eigen::MatrixXd grids and the
# Phillips path uses Eigen's unsupported FFT, so Eigen is a direct dependency.
# (It used to arrive transitively via encinowaves_vendor, which broke the
# Phillips-only build when encino is absent.)
find_package(Eigen3 REQUIRED)
# Apache-2.0 Horvath spectrum library — OPTIONAL. Present: the fft provider
# gains the EncinoWaves spectrum path (GZ_WAVES_USE_ENCINO=1). Absent: it builds
# Phillips-only with no encino dependency at all, so the FFT backend still works
# when encino isn't installed.
find_package(encinowaves_vendor QUIET)

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
  PRIVATE gz-plugin::register Eigen3::Eigen)
if(encinowaves_vendor_FOUND)
  target_compile_definitions(gz-waves-provider-fft PRIVATE GZ_WAVES_WITH_ENCINO)
  target_link_libraries(gz-waves-provider-fft
    PRIVATE encinowaves_vendor::encinowaves_vendor)
  message(STATUS "gz_waves_provider_fft: EncinoWaves spectrum ENABLED")
else()
  message(STATUS "gz_waves_provider_fft: EncinoWaves not found -> Phillips-only")
endif()

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
