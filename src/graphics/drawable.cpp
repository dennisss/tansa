#include <tansa/graphics/drawable.h>

#include <iostream>

using namespace std;


namespace tansa {
namespace graphics {

using namespace glm;

Drawable::Drawable(){
	trans = mat4(1.0f);
}

Drawable::~Drawable(){


}


void Drawable::transform(mat4 m){
	trans = m;
}

void Drawable::apply(mat4 m){
	trans = trans * m;
}






Object::Object(Shader *shader) : Drawable() {

	glGenVertexArrays(1, &this->vao);
	glBindVertexArray(this->vao);

	shader->init();

	this->shader = shader;

	this->material = NULL;

}

Object::~Object(){
	glDeleteVertexArrays(1, &this->vao);
}

void Object::draw(Camera *cam, Transform *modelview){
	glBindVertexArray(this->vao);

	glUseProgram(shader->program);

	mat4 p = cam->matrix();
	glUniformMatrix4fv(shader->uniProjAttrib, 1, GL_FALSE, glm::value_ptr(p));

	shader->setLights(cam->lights);

	mat4 mv = modelview->matrix() * trans;
	glUniformMatrix4fv(shader->uniModelViewAttrib, 1, GL_FALSE, glm::value_ptr(mv));

	vec3 eyepos = cam->position;
	glUniform3fv(shader->eyePosAttrib, 1, glm::value_ptr(eyepos));


	if(this->material != NULL){
		shader->setMaterial(*material);
	}


}

void Object::setMaterial(Material *m){
	this->material = m;
}

}
}
