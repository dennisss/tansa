cmake_minimum_required(VERSION 2.8.3)
project(tansa)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)



find_package(Eigen3 REQUIRED)

#find_package(gazebo REQUIRED)

#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_executable(listener listener.cc)
#target_link_libraries(listener ${GAZEBO_LIBRARIES} pthread)



include_directories(
	${EIGEN3_INCLUDE_DIR}
	include
)


add_executable(
	gcs

	src/gcs.cpp
	src/vehicle.cpp
)


set_target_properties(gcs PROPERTIES COMPILE_FLAGS "-std=c++11")
