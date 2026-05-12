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
target_link_libraries(gz-sim-water-visual-system
  PRIVATE
    gz-rendering::gz-rendering
    gz-common::gz-common
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
endif()

ament_package()
