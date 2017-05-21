cmake_minimum_required(VERSION 2.8)

project(mocap_camera)

if(EXISTS "/opt/vc/include/bcm_host.h")
	set(RASPBERRY_PI ON)

	# This is where raspicam puts its cmake files
	set(CMAKE_MODULE_PATH /usr/local/lib/cmake)
	find_package(raspicam REQUIRED)

	add_definitions(-DRASPBERRY_PI)
else()
	set(RASPBERRY_PI OFF)
endif()




find_package(OpenCV REQUIRED)

add_executable(
	firmware main.cpp
)

set_target_properties(firmware PROPERTIES COMPILE_FLAGS "-O3")
target_link_libraries(firmware ${OpenCV_LIBS})

if(RASPBERRY_PI)
	target_link_libraries(firmware ${raspicam_LIBS})
endif()
