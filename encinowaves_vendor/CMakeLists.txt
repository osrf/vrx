cmake_minimum_required(VERSION 3.16)
project(encinowaves_vendor)

# ----------------------------------------------------------------------------
# C++ standard
# ----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Quiet two upstream-Encino warnings (macro redefinition, unused statics
# in templates that the consumer instantiates only partially). They are
# stylistic in the upstream code and not actionable on our side.
if(NOT CMAKE_CXX_FLAGS MATCHES "-Wno-macro-redefined")
  add_compile_options(-Wno-unused-function)
endif()

# ----------------------------------------------------------------------------
# ament_cmake + dependencies
# ----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# Eigen3: header-only; FFT support via unsupported/Eigen/FFT (KissFFT
# backend by default). Apache-2.0 wraps fine over Eigen's MPL-2 / BSD-3
# combo.
find_package(Eigen3 REQUIRED NO_MODULE)

# oneTBB: parallel sweeps in Propagation.h. ament_cmake exports CMake
# config files for libtbb-dev's cmake/TBB layout.
find_package(TBB REQUIRED)

# Imath: BSD-3 math types from the OpenEXR project. Ships a CMake config
# on Debian/Ubuntu — `Imath::Imath` is the imported target. Header path is
# /usr/include/Imath which Imath::Imath sets as an INTERFACE_INCLUDE_DIR.
find_package(Imath REQUIRED)

# ----------------------------------------------------------------------------
# Library: encino_waves
# ----------------------------------------------------------------------------
# Most of EncinoWaves is header-only templates instantiated by the
# consumer. The .cpp files just carry a handful of static-singleton
# definitions and one tbb::mutex global, so a small SHARED library is
# enough.
add_library(${PROJECT_NAME} SHARED
  src/FftwWrapper.cpp
  src/Foundation.cpp
  src/Exception.cpp
  src/util/Exception.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    TBB::tbb
    Imath::Imath
)

# ----------------------------------------------------------------------------
# Install
# ----------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)

  # We don't depend on gtest itself — the smoke tests in test/ are
  # standalone driver programs that exit non-zero on failure. Wire them
  # in as ctest entries via add_test so `colcon test` picks them up.
  add_executable(test_FftwWrapper test/test_FftwWrapper.cc)
  target_link_libraries(test_FftwWrapper PRIVATE ${PROJECT_NAME})
  add_test(NAME test_FftwWrapper COMMAND test_FftwWrapper)

  add_executable(test_headers test/test_headers.cc)
  target_link_libraries(test_headers PRIVATE ${PROJECT_NAME})
  add_test(NAME test_headers COMMAND test_headers)

  add_executable(test_Propagation test/test_Propagation.cc)
  target_link_libraries(test_Propagation PRIVATE ${PROJECT_NAME})
  add_test(NAME test_Propagation COMMAND test_Propagation)
endif()

# ----------------------------------------------------------------------------
# Ament export — let downstream packages `find_package(encino_waves)`
# and pick up the include dirs, library target, and transitive deps.
# ----------------------------------------------------------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 TBB Imath)
ament_export_include_directories(include)

ament_package()
