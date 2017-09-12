
add_library(
	tansa_vision SHARED


	camera_model.cpp
	triangulation.cpp
	solve_pnp.cpp
)


target_link_libraries(
	tansa_vision

	tansa
	${CERES_LIBRARIES}
	${JPEG_LIBRARIES}
)

set_target_properties(tansa_vision PROPERTIES COMPILE_FLAGS "-std=c++1y -Wall")
