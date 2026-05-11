cmake_minimum_required(VERSION 3.8)
project(vrx_gazebo)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Hooks — register with ament so GZ_SIM_RESOURCE_PATH is set on source install/setup.bash
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
