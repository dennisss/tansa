cmake_minimum_required(VERSION 2.8.3)
project(tansa)

option(BUILD_GAZEBO "Builds support for the Gazebo simulator" ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(Cplex)

include_directories(
	${EIGEN3_INCLUDE_DIR}
	include
)

set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")
include(${CGAL_USE_FILE})


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
	src/core/data.cpp
	src/core/vehicle.cpp
	src/core/vehicle_time.cpp
	src/core/vehicle_params.cpp
	src/core/time.cpp
	src/core/messaging.cpp
	src/server/osc.cpp
	src/control/position.cpp
	src/control/hover.cpp
	src/estimation/linear_complementary.cpp
	src/trajectory/polynomial.cpp
	src/trajectory/minsnap.cpp
	src/trajectory/linear.cpp
	src/trajectory/ellipse.cpp
	src/trajectory/point.cpp
	src/trajectory/spiral.cpp
	#src/trajectory/gradual_circle.cpp
	src/mocap/mocap.cpp
	src/mocap/registration.cpp
	src/mocap/optitrack/natnet.cpp
	src/mocap/optitrack/natnet_client.cpp
	src/mocap/rigid_body_tracker.cpp
	src/jocs/routine.cpp
	src/jocs/csv.cpp
	src/jocs/jocsParser.cpp
	src/jocs/action.cpp
	src/jocs/jocsPlayer.cpp
	src/trajectory/light.cpp
	src/trajectory/strobe.cpp
	src/control/lightcontrol.cpp
	src/model/feasibility.cpp
	src/control/admittance.cpp

	src/manager/preview.cpp

	# Simulation
	src/simulate/simulation.cpp
	src/simulate/motor.cpp
	src/simulate/model.cpp
	src/simulate/multirotor.cpp
	src/simulate/firmware.cpp
	src/simulate/battery.cpp
	src/simulate/sensors/imu.cpp
	src/simulate/sensors/gps.cpp
)

target_link_libraries(
	tansa
	pthread

	sioclient
	${Boost_LIBRARIES}
	${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} # TODO: Also add CGAL_CXX_* flags
)

set_target_properties(tansa PROPERTIES COMPILE_FLAGS "-std=c++1y -Wall")


## We can optionally build with CPLEX
if(CPLEX_FOUND)
	target_include_directories(
		tansa PRIVATE
		${CPLEX_INCLUDE_DIRS}
	)

	target_link_libraries(
		tansa
		${CPLEX_LIBRARIES}
	)

	add_definitions(-DUSE_CPLEX)
endif()


######## Extensions ########
if(BUILD_GAZEBO)
	add_subdirectory(src/gazebo)

	add_definitions(-DUSE_GAZEBO)
endif()

######## Test Suite ########

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})


add_executable(
	test_tansa

	test/main.cpp
	test/core/time.cpp
	test/jocs/jocsParser.cpp
	test/mocap/registration.cpp
	test/mocap/rigid_body_tracker.cpp
)
set_target_properties(test_tansa PROPERTIES COMPILE_FLAGS "-std=c++11 -O3 -w -g")
target_link_libraries(test_tansa ${GTEST_BOTH_LIBRARIES} tansa)

add_test(TansaTests test_tansa)


######## CLI Demo ########

add_executable(
	gcs

	src/custom.cpp
	src/gcs.cpp
)

target_link_libraries(
	gcs

	pthread
	tansa
)

if(BUILD_GAZEBO)
	target_link_libraries(
		gcs
		tansa_gazebo
	)
endif()

set_target_properties(gcs PROPERTIES COMPILE_FLAGS "-std=c++1y")


###### Simple Demo ######
add_executable(
	simple

	src/simple.cpp
)

target_link_libraries(
	simple

	pthread
	tansa
)

set_target_properties(simple PROPERTIES COMPILE_FLAGS "-std=c++1y")


###### Simulation Test ######
add_executable(
	sim

	src/simulate.cpp
)

target_link_libraries(
	sim

	pthread
	tansa
)

set_target_properties(sim PROPERTIES COMPILE_FLAGS "-std=c++1y")
