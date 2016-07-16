cmake_minimum_required(VERSION 2.8.3)
project(tansa)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
)


catkin_package(


)


include_directories(
	${catkin_INCLUDE_DIRS}
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



add_executable(offboard_test
#	offboard2.cpp
	offboard_ctrl.cpp
)

target_link_libraries(
   offboard_test

   ${catkin_LIBRARIES}
)
