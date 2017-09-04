#ifndef TANSA_GRAPHICS_WINDOW_H_
#define TANSA_GRAPHICS_WINDOW_H_

#include "glm.h"
#include "transform.h"
#include "group.h"

namespace tansa {
namespace graphics {


/* Represents a drawing space either linked to a whole window or a viewport */
class Window {
public:

	static Window *Create(const char *name, glm::vec2 size = glm::vec2(1280, 720), bool visible = true);

	Group scene;
	Camera camera;

	glm::vec2 size;


	void run();

	void draw();

	void setSize(glm::vec2 size);

	inline void setBackgroundColor(glm::vec4 color){ this->backgroundColor = color; };

	inline void setActive(){ glfwMakeContextCurrent(this->window); };


private:

	Window(GLFWwindow *window);
	~Window();

	GLFWwindow *window;
	glm::vec4 backgroundColor;

};

}
}


#endif
