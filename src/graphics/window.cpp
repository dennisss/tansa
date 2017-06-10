#include <tansa/graphics/window.h>

#include <map>

using namespace std;


namespace tansa {
namespace graphics {

using namespace glm;


// All defined windows; The index corresponds to the id
static int nWindows = 0;
/*
void window_reshape(int w, int h){
	int wid = glutGetWindow();
	Window *win = windows[wid];
	win->size = vec2(w, h);

	// TODO: Allow a fixed aspect ratio as in assignment 3
	float dim = fmin(w, h);
	glViewport((w - dim) / 2.0, (h - dim) / 2.0, dim, dim);
}
*/




Window::Window(GLFWwindow *window) {
	this->window = window;

	this->backgroundColor = vec4(0.0f, 0.0f, 0.0f, 1.0f); // Default color of black
}

Window::~Window(){
	glfwDestroyWindow(this->window);

	nWindows--;
	if(nWindows == 0) {
		glfwTerminate();
	}
}


void Window::run() {
	while(!glfwWindowShouldClose(this->window)) {

		this->draw();

		glfwPollEvents(); // NOTE: This applies to all open windows
	}
}

void Window::draw() {
	this->setActive();

	vec4 cc = this->backgroundColor;
	glClearColor(cc.x, cc.y, cc.z, cc.w);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	Transform base(this->camera.view);
	this->scene.draw(&this->camera, &base);

	glfwSwapBuffers(window);
}


Window *Window::Create(const char *name, vec2 size, bool visible) {

	if(nWindows == 0) {
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
	}


	nWindows++;


	glfwWindowHint(GLFW_VISIBLE, visible? GLFW_TRUE : GLFW_FALSE);

	// TODO: http://www.glfw.org/docs/latest/context_guide.html is very useful with documenting how to do off screen windows and windows that share context with other windows
	GLFWwindow* window = glfwCreateWindow(size.x, size.y, name, NULL, NULL);

	glfwMakeContextCurrent(window);


	int res;
	glewExperimental = GL_TRUE;
	if((res = glewInit()) != GLEW_OK){
		fprintf(stderr, "GLEW Failed: %s\n", glewGetErrorString(res));
		return NULL;
	}


	glfwSwapInterval(1);


	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	Window *w = new Window(window);
	w->size = size;

	return w;
}


}
}
