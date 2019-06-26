cmake_minimum_required(VERSION 2.8.3)
project(wave_gazebo)

# We need erb to process the model erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

find_package(catkin REQUIRED)

# Model files to be processed with erb
set (model_erb_files
  world_models/ocean_waves/model.sdf.erb
)

# Process the model erb files
foreach(_erb ${model_erb_files})
  string(REGEX REPLACE ".sdf.erb" ".sdf" _model ${_erb})
  set(_model ${CMAKE_CURRENT_SOURCE_DIR}/${_model})
  add_custom_command(OUTPUT ${_model}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_model}
                     DEPENDS
                       ${CMAKE_CURRENT_SOURCE_DIR}/${_erb}
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND model_files ${_model})
endforeach()
add_custom_target(model_erb_generation ALL
  DEPENDS
    ${model_files}
)