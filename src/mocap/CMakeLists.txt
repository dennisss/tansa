

add_library(
	tansa_mocap SHARED


	mocap.cpp
	blob_detector.cpp
	rigid_body_tracker.cpp
	camera_net.cpp
	camera_node.cpp

	optitrack/natnet.cpp
	optitrack/natnet_client.cpp
)

target_link_libraries(
	tansa_mocap

	tansa
)

set_target_properties(tansa_mocap PROPERTIES COMPILE_FLAGS "-std=c++11 -O3")



find_package(OpenCV)
if(OpenCV_FOUND)
	target_link_libraries(tansa_mocap  ${OpenCV_LIBS})

	add_definitions(-DUSE_OPENCV)
endif()


######## Targets for

add_executable(
	mocap_camera

	camera_main.cpp
	interfaces/file.cpp
	interfaces/raspicam.cpp
)

set_target_properties(mocap_camera PROPERTIES COMPILE_FLAGS "-std=c++11 -O3")
target_link_libraries(mocap_camera tansa tansa_mocap)





if(EXISTS "/opt/vc/include/bcm_host.h")
	set(BUILD_RPI ON)

	# This is where raspicam puts its cmake files
	set(CMAKE_MODULE_PATH /usr/local/lib/cmake)
	find_package(raspicam REQUIRED)

	add_definitions(-DUSE_RPI)

	target_link_libraries(mocap_camera ${raspicam_LIBS})
else()
	set(BUILD_RPI OFF)
endif()
