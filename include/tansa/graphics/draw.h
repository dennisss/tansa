#ifndef TANSA_GRAPHICS_DRAW_H_
#define TANSA_GRAPHICS_DRAW_H_
// TODO: Rename that graphics.h

#include "window.h"
#include "mesh.h"
#include "polygon.h"
#include "shader.h"


namespace tansa {
namespace graphics {

	inline void Init(int *argcp, char *argv[]){
		if(!glfwInit()) {
		    // Initialization failed
			printf("GLFW Failed to initialize\n");
		}

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

		glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

		// TODO: Ensure RGBA with depth buffer
	};

}
}



#endif
