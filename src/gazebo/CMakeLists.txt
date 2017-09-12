
find_package(Boost COMPONENTS system filesystem REQUIRED)

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


########## Building the models  ###########

set(rotors_description_dir "${CMAKE_SOURCE_DIR}/lib/Firmware/Tools/sitl_gazebo/models/rotors_description")
set(scripts_dir "${CMAKE_SOURCE_DIR}/lib/Firmware/Tools/sitl_gazebo/scripts")
set(models_dir "${CMAKE_SOURCE_DIR}/config/gazebo/models")
set(design_dir "${CMAKE_SOURCE_DIR}/hardware/x340/mesh")
add_custom_command(OUTPUT ${models_dir}/x340/x340.sdf
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMAND rm -f ${models_dir}/x340/x340.sdf
	COMMAND python ${scripts_dir}/xacro.py -o  ${models_dir}/x340/x340_base.urdf  ${models_dir}/x340/x340_base.xacro rotors_description_dir:=${rotors_description_dir} models_dir:=${models_dir} design_dir:=${design_dir}
	COMMAND gz sdf -p  ${models_dir}/x340/x340_base.urdf >> ${models_dir}/x340/x340.sdf
	COMMAND rm -f ${models_dir}/x340/x340_base.urdf
)

add_custom_target(sdf ALL DEPENDS ${models_dir}/x340/x340.sdf)
