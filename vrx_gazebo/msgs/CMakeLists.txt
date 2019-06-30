find_package(Protobuf REQUIRED)
set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()
set (msgs
  light_buoy_colors.proto
  dock_placard.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(light_buoy_colors_msgs SHARED ${PROTO_SRCS})
target_link_libraries(light_buoy_colors_msgs ${PROTOBUF_LIBRARY})

add_library(dock_placard_msgs SHARED ${PROTO_SRCS})
target_link_libraries(dock_placard_msgs ${PROTOBUF_LIBRARY})
