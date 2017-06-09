#ifndef TANSA_GRAPHICS_WINDOW_H_
#define TANSA_GRAPHICS_WINDOW_H_

/*
	Windows are multiplexed by their ids

	glutGetWindow() will tell which window each callback is currently in

	They are defined by a  (pointer to a) scene and a camera

	Scene -> Objects and lights in the world
	Camera -> How the world is observed

*/

#include "glm.h"
#include "transform.h"
#include "group.h"

namespace tansa {
namespace graphics {


/* Represents a drawing space either linked to a whole window or a viewport */
class Window {
public:

	static Window *Create(const char *name, glm::vec2 size = glm::vec2(512,512));

	Group scene;
	Camera camera;

	glm::vec2 size;


	void run();

	void draw();

	inline void setBackgroundColor(glm::vec4 color){ this->backgroundColor = color; };

	inline void setActive(){ glfwMakeContextCurrent(this->window); };

	GLFWwindow *window;


private:

	Window(GLFWwindow *window);
	~Window();

	glm::vec4 backgroundColor;

};


// Used to have distributed idle function
// Private function, do not use directly
void window_idle();;

}
}


#endif
