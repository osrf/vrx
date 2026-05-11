cmake_minimum_required(VERSION 3.8)
project(vrx_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
