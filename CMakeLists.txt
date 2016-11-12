cmake_minimum_required(VERSION 2.8.3)
project(tansa)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
	${EIGEN3_INCLUDE_DIR}
	include
)


######## Headers and libraries for prebuilt socket.io client ########

include_directories(
	lib/socket.io-client-cpp/build/include
)

link_directories(
	build_socketio_cpp
)


######## Main Tansa Library ########

add_library(
	tansa SHARED

	src/core/core.cpp
	src/core/vehicle.cpp
	src/core/vehicle_time.cpp
	src/core/time.cpp
	src/core/messaging.cpp
	src/control/position.cpp
	src/control/hover.cpp
	src/estimation/linear_complementary.cpp
	src/trajectory/polynomial.cpp
	src/trajectory/linear.cpp
	src/trajectory/circle.cpp
	src/trajectory/point.cpp
	src/mocap/mocap.cpp
	src/mocap/registration.cpp
	src/mocap/optitrack/ClientCore.cpp
	src/mocap/optitrack/NatNetClient.cpp
    src/jocs/jocsParser.cpp
	src/jocs/action.cpp
	src/jocs/jocsPlayer.cpp
)

target_link_libraries(
	tansa

	pthread

	sioclient
	boost_system
	${Boost_LIBRARIES}
)

set_target_properties(tansa PROPERTIES COMPILE_FLAGS "-std=c++1y -Wall")



######## Extensions ########

add_subdirectory(src/gazebo)


######## Test Suite ########

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})


add_executable(
	test_tansa

	test/main.cpp
	test/core/time.cpp
	test/mocap/registration.cpp
)
set_target_properties(test_tansa PROPERTIES COMPILE_FLAGS "-std=c++11 -O3 -w -g")
target_link_libraries(test_tansa ${GTEST_BOTH_LIBRARIES} tansa)

add_test(TansaTests test_tansa)


######## CLI Demo ########

add_executable(
	gcs

	src/gcs.cpp

)

target_link_libraries(
	gcs

	pthread
	tansa
	tansa_gazebo
)

set_target_properties(gcs PROPERTIES COMPILE_FLAGS "-std=c++1y")



########## Targets for Gazebo Simulation ###########


set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/lib/Firmware/Tools/sitl_gazebo/models/rotors_description")
set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/lib/Firmware/Tools/sitl_gazebo/scripts")
set(models_dir "${CMAKE_CURRENT_SOURCE_DIR}/config/gazebo/models")
set(design_dir "${CMAKE_CURRENT_SOURCE_DIR}/design/x340")
add_custom_command(OUTPUT ${models_dir}/x340/x340.sdf
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMAND rm -f ${models_dir}/x340/x340.sdf
	COMMAND python ${scripts_dir}/xacro.py -o  ${models_dir}/x340/x340_base.urdf  ${models_dir}/x340/x340_base.xacro rotors_description_dir:=${rotors_description_dir} models_dir:=${models_dir} design_dir:=${design_dir}
	COMMAND gz sdf -p  ${models_dir}/x340/x340_base.urdf >> ${models_dir}/x340/x340.sdf
 	COMMAND rm -f ${models_dir}/x340/x340_base.urdf
)

add_custom_target(sdf ALL DEPENDS ${models_dir}/x340/x340.sdf)
