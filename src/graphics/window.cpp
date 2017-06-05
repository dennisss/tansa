#include <tansa/graphics/window.h>

#include <map>

using namespace std;


namespace tansa {
namespace graphics {

using namespace glm;


// All defined windows; The index corresponds to the id
static map<int, Window *> windows;



void window_display(){

	int wid = glutGetWindow();
	Window *w = windows[wid];

	vec4 cc = w->backgroundColor;
	glClearColor(cc.x, cc.y, cc.z, cc.w);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	Transform base(w->camera.view);
	w->scene.draw(&w->camera, &base);

	glutSwapBuffers();
}

void window_keyboard(unsigned char key, int x, int y){


}

void window_idle(){

	for(const pair<int, Window *> &p : windows){

		Window *w = p.second;

		if(w->onIdle != NULL){
			w->setActive();
			bool res = w->onIdle();

			if(res)
				glutPostRedisplay();
		}
	}
}


void window_reshape(int w, int h){
	int wid = glutGetWindow();
	Window *win = windows[wid];
	win->size = vec2(w, h);

	// TODO: Allow a fixed aspect ratio as in assignment 3
	float dim = fmin(w, h);
	glViewport((w - dim) / 2.0, (h - dim) / 2.0, dim, dim);
}

void window_menuevent(int option){


}



Window::Window(int id){
	this->id = id; windows[id] = this;

	this->backgroundColor = vec4(0.0f, 0.0f, 0.0f, 1.0f); // Default color of black

	this->onIdle = NULL;

	glutDisplayFunc(window_display);
//	glutReshapeFunc(window_reshape);
//	glutKeyboardFunc(keyboard);

}
Window::~Window(){
	glutDestroyWindow(this->id);
	windows.erase(this->id);
}


Window *Window::Create(char *name, vec2 size){

	glutInitWindowSize(size.x, size.y);

	int id = glutCreateWindow(name), res;
	glewExperimental = GL_TRUE;
	if((res = glewInit()) != GLEW_OK){
		fprintf(stderr, "GLEW Failed: %s\n", glewGetErrorString(res));
		return NULL;
	}

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	Window *w = new Window(id);
	w->size = size;

	return w;
}

Window *Window::sub(int x, int y, int width, int height){

	int w = glutCreateSubWindow(id, x, y, width, height), res;

	// TODO: This code is redundant

	glewExperimental = GL_TRUE;
	if((res = glewInit()) != GLEW_OK){
		fprintf(stderr, "GLEW Failed: %s\n", glewGetErrorString(res));
		return NULL;
	}

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);


	Window *win = new Window(w);
	win->size = vec2(width, height);

	return win;
}



/*
void processMenuEvents()
{
	switch (option)
	{
		case RED : red = 1.0; green = 0.0; blue = 0.0; break;
		case GREEN : red = 0.0; green = 1.0; blue = 0.0; break;
		case BLUE : red = 0.0; green = 0.0; blue = 1.0; break;
		case WHITE : red = 1.0; green = 1.0; blue = 1.0; break;
	}
	glutPostRedisplay();
}
*/


void Window::addMenu(MenuItem *root, MenuCallback callback){

	// TODO: Set current window


	// Make root menu
	int menu = glutCreateMenu(callback);

	vector<MenuItem *> path = {root};
	vector<int> menus = {menu};

	//vector<int> counts = {root->nchildren};

	for(MenuItem *cur = root + 1; path.size() > 0; ++cur){

		MenuItem *parent = path.back();

		if(cur->nchildren == 0){
			glutAddMenuEntry(cur->name, cur->value);
		}
		else{
			int sub = glutCreateMenu(callback);

			path.push_back(cur);
			menus.push_back(sub);
		}

		parent->nchildren--;


		while(path.size() > 0 && path.back()->nchildren == 0){

			MenuItem *i = path.back(); path.pop_back();
			int m = menus.back(); menus.pop_back();

			// Pop and attach to previous menu
			if(path.size() > 0){
				glutSetMenu(menus.back());
				glutAddSubMenu(i->name, m);
			}
		}

	}



//	int menu,submenu;
//	submenu = glutCreateMenu(processMenuEvents);
//	glutAddMenuEntry("Red",RED);
//	glutAddMenuEntry("Blue",BLUE);
//	glutAddMenuEntry("Green",GREEN);
//	menu = glutCreateMenu(processMenuEvents);
//	glutAddMenuEntry("White",WHITE);
//	glutAddSubMenu("RGB Menu",submenu);


	glutSetMenu(menu);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}


}
}
