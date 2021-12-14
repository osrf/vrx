cmake_minimum_required(VERSION 3.5)
project(wave_gazebo)

# We need erb to process the model erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

ament_export_dependencies(xacro)

# Model files to be processed with erb
set (xacro_erb_files
  world_models/ocean_waves/model.xacro.erb
)
# Process the xacro erb files
foreach(_erb ${xacro_erb_files})
  string(REGEX REPLACE ".xacro.erb" ".xacro" _model ${_erb})
  set(_model ${CMAKE_CURRENT_SOURCE_DIR}/${_model})
  add_custom_command(OUTPUT ${_model}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_model}
                     DEPENDS
                       ${CMAKE_CURRENT_SOURCE_DIR}/${_erb}
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND model_files ${_model})
endforeach()

install(DIRECTORY world_models/
  DESTINATION share/${PROJECT_NAME}/world_models)

add_custom_target(model_erb_generation ALL
  DEPENDS
    ${model_files}
)

# Generate world files from xacro and install
if(ADD_XACRO)
  xacro_add_files(
    worlds/ocean.world.xacro
    worlds/ocean_buoys.world.xacro
    worlds/ocean_wamv.world.xacro
    INSTALL DESTINATION worlds
  )
endif()

ament_package()
