cmake_minimum_required(VERSION 2.8.3)
project(gazebo_spawn)

set(TANSA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../")
set(CMAKE_MODULE_PATH ${TANSA_ROOT}/cmake)
find_package(Eigen3 REQUIRED)

include_directories(
	${TANSA_ROOT}/include
	${EIGEN3_INCLUDE_DIR}
)

link_directories(
	${TANSA_ROOT}/build
	${TANSA_ROOT}/build/src/gazebo
)


add_executable(
	spawner

	main.cpp
)


target_link_libraries(
	spawner

	tansa
	tansa_gazebo
)

set_target_properties(spawner PROPERTIES COMPILE_FLAGS "-std=c++1y -Wall")
