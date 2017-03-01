
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
	if(ITR MATCHES ".*gazebo-[0-9.]+$")
		set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
	endif()
endforeach()

set(msgs
	spawn.proto
	${PROTOBUF_IMPORT_DIRS}/vector3d.proto
	${PROTOBUF_IMPORT_DIRS}/header.proto
	${PROTOBUF_IMPORT_DIRS}/time.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(tansa_gazebo_msgs SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(tansa_gazebo_msgs ${PROTOBUF_LIBRARIES} ${GAZEBO_LIBRARIES})
