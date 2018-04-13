#ifdef BUILD_GRAPHICS
#include "virtual.h"

#include <tansa/time.h>

#include <iostream>
using namespace std;


namespace tansa {

using namespace graphics;
using namespace graphics::glm;


/*
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {

	if(action != GLFW_PRESS && action != GLFW_REPEAT) {
		return;
	}


	if(key == GLFW_KEY_UP) radius += 0.5;
	if(key == GLFW_KEY_DOWN) radius -= 0.5;

	if(key == GLFW_KEY_A) angle += 0.05;
	if(key == GLFW_KEY_D) angle -= 0.05;
	if(key == GLFW_KEY_S) height -= 0.05;
	if(key == GLFW_KEY_W) height += 0.05;

	if(key == GLFW_KEY_P) {
		if(mode == MODE_PERSPECTIVE)
			mode = MODE_PARALLEL;
		else
			mode = MODE_PERSPECTIVE;
	}

	update_camera();
}
*/

static void shrink(char *in, char *out, unsigned width, unsigned height, unsigned depth, unsigned factor) {


	unsigned newWidth = width / factor,
			 newHeight = height / factor;

	char *inP = in;
	char *outP = out;

	for(int i = 0; i < newHeight; i++) {
		for(int j = 0; j < newWidth; j++) {
			for(int d = 0; d < depth; d++) {
				outP[d] = inP[d];
			}


			inP += depth * factor;
			outP += depth;
		}

		inP += width * (factor - 1) * depth;
	}


}

VirtualImagingInterface::VirtualImagingInterface(const CameraModel &cam) {

	bool visible = true;

	window = Window::Create("Camera View", { (GLfloat) cam.width, (GLfloat) cam.height }, visible);

	if(!visible) {
		glViewport(0, 0, (GLfloat) cam.width, (GLfloat) cam.height); // On Mac at least, the pixel coordinates aren't the same as the screen coordinates
	}
	else {
		// TODO: This code assumes that the OS is being scaled with a scaling of 2 (like on me Mac)
		scale = 0.5;
		window->setSize({ (GLfloat) cam.width / 2, (GLfloat) cam.height / 2 });
	}

	Shader *shader = Shader::Gouraud();

	Material *material = new Material(); // TODO: This needs to be dynamically allocated
	material->shininess = 0.5;
	material->diffuse = vec3(0.2, 0.2, 0.2);
	material->ambient = vec3(0.2, 0.2, 0.9);
	material->specular = vec3(0, 0, 0);


	// Add two lights in the camera frame
	window->camera.lights.resize(1);
	window->camera.lights[0].position = vec4(0, 0, -1, 0);
	window->camera.lights[0].diffuse = vec3(0.4,0.4,0.4);
	window->camera.lights[0].ambient = vec3(1,1,1);
	window->camera.lights[0].specular = vec3(1,1,1);

	//	win->camera.lights[1].position = vec4(1, 0, 0, 0);
	//	win->camera.lights[1].diffuse = vec3(0.2,0.2,0.2);


	Mesh *mesh = Mesh::read("./hardware/x260/mesh/body.stl", shader);
	float f = 1.0/1000.0;
	mesh->transform(glm::scale({ f, f, f }));
	mesh->setMaterial(material);
	window->scene.addObject(mesh); // TODO: Add this back in


	Mesh *sphere = SphereHelper({ 0, 0.13, 0.04 }, 0.005, 50, 50, shader);

	/*
	Vector3d a = cam.projection()*Vector4d(0, 0.13, 0.04, 1.0);
	cout << "TARGET: " << a.x() / a.z() << " " << a.y() / a.z() << endl;
	*/

	Material *reflective = new Material();
	reflective->shininess = 0.9;
	reflective->diffuse = vec3(1, 1, 1);
	reflective->ambient = vec3(1, 1, 1);
	reflective->specular = vec3(1, 1, 1);
	sphere->setMaterial(reflective);
	window->scene.addObject(sphere);


	Mesh *sphere2 = SphereHelper({ 0, -0.13, 0.04 }, 0.005, 50, 50, shader);
	sphere2->setMaterial(reflective);
	window->scene.addObject(sphere2);

	Mesh *sphere3 = SphereHelper({ 0.13, 0, 0.04 }, 0.005, 50, 50, shader);
	sphere3->setMaterial(reflective);
	window->scene.addObject(sphere3);

	this->camera = cam;
	setup_camera();


}

VirtualImagingInterface::~VirtualImagingInterface() {


}



/*
	Tansa will obide by the OpenCV camera model which has the camera looking down the +z space
	- All code outside of the OpenGL stuff will use the general pin hole camera model projection matrix for doing any


	For a physical camera modeled by:
	f_x 0 c_x
	0 f_y c_y
	0  0  1

	3*4
	4*1




	The equivalent OpenGL projection matrix is:
	Note: OpenGL cameras start looking at -z with +x to the right and +y up and the origin in the center of frame
	Note: For OpenCV/Computer vision, our camera looks in the +z with +x to the right and +y down

	2.0 * fx / width	0						1.0 - 2.0 * cx / width				0
	0					-2.0 * fy / height		2.0 * cy / height - 1.0				0
	0					0						(zfar + znear) / (znear - zfar)		2*(zfar*znear)/(znear-zfar)
	0					0						-1.0								0

	Note: The last two rows are equivalent to the usual OpenGL formulation


	Why the above matrix:
	- width maps to -1 to 1
		- so scaling from width to 2 is a factor of 1 / (width / 2) = 2 / width

	- flip y to be pointing up
		- it points down in opencv

	- Negate z to be looking down -z instead of +z
		- This is why the third row is negated from usual


	LATEST:
	This matrix is pretty much the usual OpenGL matrix P but is instead :

	Pnew = P* [ 1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1 ] // Basically inverting y and z before projecting

*/
/**
 * Generates an OpenGL compatible projection matrix simulating a real camera
 */
glm::mat4 cameraModel(float fx, float fy, float cx, float cy, float height, float width, float zNear, float zFar) {

	return glm::mat4(
		(2.0*fx/width), 0, -(1.0 - 2.0*cx/width), 0,
		0, (-2.0*fy/height), -(2.0*cy/height - 1.0), 0,
		0, 0, ((zFar + zNear) / (zFar - zNear)), (-2.0*(zFar*zNear)/(zFar - zNear)),
		0, 0, 1.0, 0
	);
}


glm::mat4 eig2glm(const Matrix4d &m) {
	return glm::mat4(
		m(0, 0), m(0, 1), m(0, 2), m(0, 3),
		m(1, 0), m(1, 1), m(1, 2), m(1, 3),
		m(2, 0), m(2, 1), m(2, 2), m(2, 3),
		m(3, 0), m(3, 1), m(3, 2), m(3, 3)
	);
}

void VirtualImagingInterface::setup_camera() {

	// This is roughly equivalent to the max distance the camera can see away from it
	float radius = 0.5; // TODO:

	// TODO: Now we just need a common way to express rotation between the two frames

	// The scaling is needed as glReadPixels reads it upside down
	// TODO: We really need to verify that the scaling is not needed
	window->camera.proj = cameraModel(scale*camera.fx, scale*camera.fy, scale*camera.cx, scale*camera.cy, scale*camera.height, scale*camera.width, 0.1, 6); // glm::perspective(M_PI/2.0, 1280.0 / 720.0, 0.1, radius + 5);

	//window->camera.view = glm::lookAt(glm::vec3(camera.position.x(), camera.position.y(), 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 1));
	// Make rotation homogenous
	Matrix4d r = Matrix4d::Identity();
	r.block<3,3>(0, 0) = camera.rotation.inverse();

	window->camera.view = eig2glm(r) * glm::translate(vec3(-camera.position.x(), -camera.position.y(), -camera.position.z()));


	/*
	// Testing projection of a point

	glm::vec4 v(0, -0.13, 0.04, 1);

	v = window->camera.proj * window->camera.view * v;
	cout << ((camera.width / 2)*(v.x / v.w) + camera.cx) << " " << (camera.cy - (camera.height / 2)*(v.y / v.w)) << " " << v.z << " " << v.w << endl;
	*/

}


void VirtualImagingInterface::start() {

	img.data = (uint8_t *) malloc(camera.width * camera.height);
	img.width = camera.width;
	img.height = camera.height;

	thread = std::thread(virtual_imaging_thread, this);
}


void VirtualImagingInterface::stop() {


	free(img.data);
}

ImageSize VirtualImagingInterface::getSize() {
	ImageSize s;
	s.width = camera.width;
	s.height = camera.height;
	return s;
}


void take_image(Image *img) {

	GLint dims[4] = {0};
	glGetIntegerv(GL_VIEWPORT, dims);
	GLint width = dims[2];
	GLint height = dims[3];

	// TODO: Possibly investigate doing: https://stackoverflow.com/questions/25127751/opengl-read-pixels-faster-than-glreadpixels
	glReadPixels(0, 0, width, height, GL_BLUE /* GL_RGB */, GL_UNSIGNED_BYTE, img->data);

	// glReadPixels reads with the start of the buffer at the bottom-left corner of the image,
	// so this flips it vertically in place
	img->flipY();
}



void virtual_imaging_thread(VirtualImagingInterface *inst) {

	Rate rate(2);
	while(true) {
		inst->window->draw();

		take_image(&inst->img);

		MocapCameraImage *msg = new MocapCameraImage();
		msg->image = &inst->img;
		inst->publish(msg);

		//glfwPollEvents();
		rate.sleep();
	}

}

}

#endif
