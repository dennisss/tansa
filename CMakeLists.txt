cmake_minimum_required(VERSION 2.8.3)
project(tansa)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
	gazebo_msgs
	angles
	eigen_conversions
	mavros_msgs
)

find_package(Eigen3)



add_message_files(
	FILES
	PointArray.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)



catkin_package(
	CATKIN_DEPENDS message_runtime

)



include_directories(
	${catkin_INCLUDE_DIRS}
	include
)


add_executable(mocap_node
	src/mocap/mocap_node

	src/mocap/optitrack/NatNetClient.cpp
	src/mocap/optitrack/ClientCore.cpp
)
target_link_libraries(
   mocap_node

   ${catkin_LIBRARIES}
)
add_dependencies(mocap_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_dependencies(your_program ${catkin_EXPORTED_TARGETS})



add_executable(mocap_gazebo_node
	src/mocap/mocap_gazebo_node.cpp
)
target_link_libraries(mocap_gazebo_node
   ${catkin_LIBRARIES}
)







#add_executable(offboard_test
#	offboard2.cpp
#	src/offboard_ctrl.cpp
#)

#target_link_libraries(
#   offboard_test
#
#   ${catkin_LIBRARIES}
#)

add_executable(offb_node
	src/offb_node.cpp
)
set_target_properties(offb_node PROPERTIES COMPILE_FLAGS "-std=c++11 -g")
target_link_libraries(offb_node
	${catkin_LIBRARIES}
)

add_executable(state_log
	src/state_log.cpp
)
target_link_libraries(state_log
	${catkin_LIBRARIES}
)


add_executable(light_node
	src/lighting/light_node.cpp
)
target_link_libraries(light_node
	${catkin_LIBRARIES}
)
