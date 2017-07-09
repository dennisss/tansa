find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


add_library(
	tansa_beacon

	calibration.cpp


)

target_link_libraries(tansa_beacon ${CERES_LIBRARIES})
set_target_properties(tansa_beacon PROPERTIES COMPILE_FLAGS "-std=c++1y")
