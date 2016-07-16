cmake_minimum_required(VERSION 2.8)
project(zed)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


find_package(OpenCV REQUIRED)

add_executable(matcher
	match.cpp
)

set_target_properties(matcher PROPERTIES COMPILE_FLAGS "-std=c++11 -O3 -w -g")
target_link_libraries(matcher ${OpenCV_LIBS})
