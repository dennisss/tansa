
find_package(gazebo REQUIRED)

include_directories(
	${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_subdirectory(msgs)


add_library(
	tansa_gazebo SHARED

	gazebo.cpp
)

target_link_libraries(
	tansa_gazebo

	tansa
	tansa_gazebo_msgs
	${GAZEBO_LIBRARIES}
)



######## Plugin for preloading into a Gazebo world ########

add_library(
	gazebo_swarm SHARED

	swarm_plugin.cpp
)

add_dependencies(gazebo_swarm tansa_gazebo_msgs)
target_link_libraries(gazebo_swarm tansa_gazebo_msgs ${GAZEBO_LIBRARIES})
target_include_directories(
	gazebo_swarm PRIVATE

	${CMAKE_CURRENT_BINARY_DIR}/msgs # For the protocol buffer messages
)
