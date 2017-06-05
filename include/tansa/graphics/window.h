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

typedef void (*MenuCallback)(int value);
typedef bool (*IdleCallback)();

struct MenuItem {
	char *name;
	int value;
	int nchildren;
};



/* Represents a drawing space either linked to a whole window or a viewport */
class Window {

	friend void window_display();
	friend void window_idle();

public:

	static Window *Create(char *name, glm::vec2 size = glm::vec2(512,512));

	Window *sub(int x, int y, int width, int height);
	//Window *sub();


	Group scene;
	Camera camera;

	glm::vec2 size;


	void addMenu(MenuItem *root, MenuCallback callback);
	inline void setBackgroundColor(glm::vec4 color){ this->backgroundColor = color; };

	inline void setActive(){ glutSetWindow(this->id); };

	inline void setIdle(IdleCallback c){ this->onIdle = c; };

private:

	Window(int id);
	~Window();


	int id;

	glm::vec4 backgroundColor;


	IdleCallback onIdle;

};


// Used to have distributed idle function
// Private function, do not use directly
void window_idle();;

}
}


#endif
