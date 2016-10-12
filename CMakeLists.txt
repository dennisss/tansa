cmake_minimum_required(VERSION 2.8.3)
project(tansa)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)



find_package(Eigen3 REQUIRED)

find_package(gazebo REQUIRED)


include_directories(
	${GAZEBO_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	include
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(
	gcs

	src/gcs.cpp
	src/core/core.cpp
	src/core/vehicle.cpp
	src/core/vehicle_time.cpp
	src/core/time.cpp
	src/core/gazebo.cpp
	src/control/position.cpp
	src/control/hover.cpp
	src/trajectory/polynomial.cpp
	src/trajectory/linear.cpp
	src/trajectory/circle.cpp
	src/mocap/mocap.cpp
	src/mocap/registration.cpp
	src/mocap/optitrack/ClientCore.cpp
	src/mocap/optitrack/NatNetClient.cpp
    src/jocs/jocsParser.cpp

)

target_link_libraries(
	gcs

	${GAZEBO_LIBRARIES}
	pthread
)


set_target_properties(gcs PROPERTIES COMPILE_FLAGS "-std=c++11")
