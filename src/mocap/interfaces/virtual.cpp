#ifdef BUILD_GRAPHICS
#include "virtual.h"

#include <tansa/time.h>

using namespace std;


namespace tansa {

using namespace graphics;
using namespace graphics::glm;

VirtualImagingInterface::VirtualImagingInterface() {

	window = Window::Create("Camera View", {1280, 720}, false);
	glViewport(0, 0, 1280, 720); // On Mac at least, the pixel coordinates aren't the same as the screen coordinates

	Shader *shader = Shader::Gouraud();

	Material *material = new Material(); // TODO: This needs to be dynamically allocated
	material->shininess = 0.5;
	material->diffuse = vec3(0.2, 0.2, 0.2);
	material->ambient = vec3(0.2, 0.2, 0.2);
	material->specular = vec3(0, 0, 0);


	// Add two lights in the camera frame
	window->camera.lights.resize(1);
	window->camera.lights[0].position = vec4(0, 0, -1, 0);
	window->camera.lights[0].diffuse = vec3(0.4,0.4,0.4);
	window->camera.lights[0].ambient = vec3(1,1,1);
	window->camera.lights[0].specular = vec3(1,1,1);

	//	win->camera.lights[1].position = vec4(1, 0, 0, 0);
	//	win->camera.lights[1].diffuse = vec3(0.2,0.2,0.2);


	Mesh *mesh = Mesh::read("../hardware/x260/mesh/body.stl", shader);
	float f = 1.0/1000.0;
	mesh->transform(glm::scale({ f, f, f }));
	mesh->setMaterial(material);
	window->scene.addObject(mesh);


	Mesh *sphere = SphereHelper({ 0, 0.13, 0.04 }, 0.01, 50, 50, shader);

	Material *reflective = new Material();
	reflective->shininess = 0.9;
	reflective->diffuse = vec3(1, 1, 1);
	reflective->ambient = vec3(1, 1, 1);
	reflective->specular = vec3(1, 1, 1);
	sphere->setMaterial(reflective);
	window->scene.addObject(sphere);


	Mesh *sphere2 = SphereHelper({ 0, -0.13, 0.04 }, 0.01, 50, 50, shader);
	sphere2->setMaterial(reflective);
	window->scene.addObject(sphere2);

	Mesh *sphere3 = SphereHelper({ 0.13, 0, 0.04 }, 0.01, 50, 50, shader);
	sphere3->setMaterial(reflective);
	window->scene.addObject(sphere3);


	setup_camera();


}

VirtualImagingInterface::~VirtualImagingInterface() {


}


void VirtualImagingInterface::setup_camera() {

	static float angle = 0;
	static float radius = 0.5;
	static float height = 0.2;

	float x = radius*cos(angle);
	float y = radius*sin(angle);


	// The scaling is needed as glReadPixels reads it upside down
	window->camera.proj = glm::scale(vec3(1, -1, 1)) * glm::perspective(M_PI/2.0, 1280.0 / 720.0, 0.1, radius + 5);
	window->camera.view = glm::lookAt(glm::vec3(x, y, height), glm::vec3(0, 0, 0), glm::vec3(0, 0, 1));
}


void VirtualImagingInterface::start() {

	img.data = (uint8_t *) malloc(1280*720);
	img.width = 1280;
	img.height = 720;

	thread = std::thread(virtual_imaging_thread, this);
}


void VirtualImagingInterface::stop() {


}

ImageSize VirtualImagingInterface::getSize() {
	ImageSize s;
	s.width = 1280;
	s.height = 720;
	return s;
}


void take_image(Image *img) {

	GLint dims[4] = {0};
	glGetIntegerv(GL_VIEWPORT, dims);
	GLint width = dims[2];
	GLint height = dims[3];

	// TODO: Possibly investigate doing: https://stackoverflow.com/questions/25127751/opengl-read-pixels-faster-than-glreadpixels
	glReadPixels(0, 0, width, height, GL_BLUE /* GL_RGB */, GL_UNSIGNED_BYTE, img->data);



}

void virtual_imaging_thread(VirtualImagingInterface *inst) {

	Rate rate(30);
	while(true) {
		inst->window->draw();

		take_image(&inst->img);


		MocapCameraImage msg;
		msg.image = &inst->img;
		inst->publish(msg);

		//glfwPollEvents();
		rate.sleep();
	}

}

}

#endif
