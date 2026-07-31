cmake_minimum_required(VERSION 3.16)
project(vrx_blueboat)

find_package(ament_cmake REQUIRED)

# All BlueBoat models install under one directory, and that is load-bearing:
# the models cross-reference each other's meshes with
# file://<model_dir>/meshes/..., which resolves only because that single
# directory is on GZ_SIM_RESOURCE_PATH. Splitting them across packages, or
# nesting them further, breaks every cross-model URI.
install(
  DIRECTORY share/models
  DESTINATION share/${PROJECT_NAME}
)

# glb_stats.py: reports triangle/vertex counts, textures and bounding-box
# extents for .glb files. Used to catch bad mesh scale before it reaches sim.
install(
  PROGRAMS tools/glb_stats.py
  DESTINATION lib/${PROJECT_NAME}
)

# GZ_SIM_RESOURCE_PATH hook (models only - this package builds nothing).
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
