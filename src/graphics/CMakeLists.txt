

add_library(
	tansa_graphics SHARED

	drawable.cpp
	group.cpp
	mesh.cpp
	mesh_smf.cpp
	mesh_stl.cpp
	patch.cpp
	polygon.cpp
	shader.cpp
	transform.cpp
	utils.cpp
	window.cpp
	shapes.cpp
)

target_link_libraries(
	tansa_graphics

	${OPENGL_LIBRARIES}
	${GLEW_LIBRARIES}
	glfw
)

target_include_directories(
	tansa_graphics PRIVATE

	${OPENGL_INCLUDE_DIRS}
	${GLEW_INCLUDE_DIRS}
)

set_target_properties(tansa_graphics PROPERTIES COMPILE_FLAGS "-std=c++1y")
