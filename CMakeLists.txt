cmake_minimum_required(VERSION 2.8.3)
project(tansa)

######## Dependency Resolution ########
# Depending on which libraries are available on the system, we will build in support in support for them

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Most of these are intentionally optional although not including some will limit the available functionality
find_package(Eigen3 REQUIRED)
find_package(OpenGL)
find_package(glfw3 3.2)
find_package(GLEW)
find_package(CGAL)
find_package(Cplex)
find_package(OpenCV)
find_package(gazebo)
find_package(Ceres)
find_package(JPEG)
# TODO: Also use glog


include_directories(
	${EIGEN3_INCLUDE_DIR}
	include
)


if(JPEG_FOUND)
	include_directories(${JPEG_INCLUDE_DIR})
endif()

if(Ceres_FOUND)
	include_directories(${CERES_INCLUDE_DIRS})
	add_definitions(-DUSE_CERES)
endif()

if(OpenCV_FOUND)
	add_definitions(-DUSE_OPENCV)
endif()

if(gazebo_FOUND)
	set(BUILD_GAZEBO ON)
endif()

if((OpenGL_FOUND) AND (glfw3_FOUND) AND (GLEW_FOUND))
	set(BUILD_GRAPHICS ON)
	add_definitions(-DBUILD_GRAPHICS)
endif()

if(CGAL_FOUND)
	set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")
	include(${CGAL_USE_FILE})
	add_definitions(-DUSE_CGAL)
endif()


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
	src/core/context.cpp
	src/core/net.cpp
	src/core/data.cpp
	src/core/vehicle.cpp
	src/core/vehicle_time.cpp
	src/core/vehicle_params.cpp
	src/core/time.cpp
	src/core/messaging.cpp
	src/algorithm/assignment_solver.cpp
	src/algorithm/rigid_transform.cpp
	src/algorithm/rigid_point_correspondence_solver.cpp
	src/server/osc.cpp
	src/control/position.cpp
	src/estimation/linear_complementary.cpp
	src/trajectory/polynomial.cpp
	src/trajectory/minsnap.cpp
	src/trajectory/linear.cpp
	src/trajectory/ellipse.cpp
	src/trajectory/point.cpp
	src/trajectory/spiral.cpp
	#src/trajectory/gradual_circle.cpp
	src/jocs/routine.cpp
	src/jocs/csv.cpp
	src/jocs/jocsParser.cpp
	src/jocs/action.cpp
	src/jocs/jocsPlayer.cpp
	src/trajectory/light.cpp
	src/trajectory/strobe.cpp
	src/control/lightcontrol.cpp
	src/model/feasibility.cpp
	#src/control/admittance.cpp

	src/manager/preview.cpp
	src/manager/calibration.cpp

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
)

set_target_properties(tansa PROPERTIES COMPILE_FLAGS "-std=c++1y -Wall")



if(CGAL_FOUND)
	target_link_libraries(
		tansa
		${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} # TODO: Also add CGAL_CXX_* flags
	)
endif()


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

if(BUILD_GRAPHICS)
	add_subdirectory(src/graphics)
endif()

if(BUILD_GAZEBO)
	add_subdirectory(src/gazebo)

	add_definitions(-DUSE_GAZEBO)
endif()

add_subdirectory(src/vision)

add_subdirectory(src/mocap)

add_subdirectory(src/beacon)

######## Test Suite ########

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})


add_executable(
	test_tansa

	test/main.cpp
	test/core/time.cpp
	test/algorithm/assignment_solver.cpp
	test/algorithm/disjoint_sets.cpp
	test/algorithm/rigid_point_correspondence_solver.cpp
	test/jocs/jocsParser.cpp
	test/mocap/rigid_body_tracker.cpp
	test/mocap/calibration.cpp
)
set_target_properties(test_tansa PROPERTIES COMPILE_FLAGS "-std=c++11 -O3 -w -g")
target_link_libraries(
	test_tansa

	${GTEST_BOTH_LIBRARIES}
	tansa
	tansa_vision
)

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
	tansa_mocap
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


###### Custom Mocap Testing ######
add_executable(
	mocap_test

	src/mocap_test.cpp
)

target_link_libraries(
	mocap_test

	pthread
	tansa
	tansa_mocap
)

set_target_properties(mocap_test PROPERTIES COMPILE_FLAGS "-std=c++1y")


###### Other utilities ######

add_subdirectory(scripts/camera_calibration)
