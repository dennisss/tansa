#include <tansa/graphics/shader.h>

#include <stdio.h>


#include <vector>
#include <fstream>
#include <string>
#include <iostream>

using namespace std;

namespace tansa {
namespace graphics {

void Shader::init(){
	glEnableVertexAttribArray(posAttrib);
	glEnableVertexAttribArray(normalAttrib);
	glEnableVertexAttribArray(colorAttrib);
}


Shader *Shader::Load(ifstream &&vertexFile, ifstream &&fragmentFile){

	string vsrc((std::istreambuf_iterator<char>(vertexFile) ), (std::istreambuf_iterator<char>()));
	string fsrc((std::istreambuf_iterator<char>(fragmentFile) ), (std::istreambuf_iterator<char>()));

	return Shader::Load(vsrc.c_str(), fsrc.c_str());
}

Shader *Shader::Load(const char *vertexSrc, const char *fragmentSrc){
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexSrc, NULL);
	glCompileShader(vertexShader);

	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentSrc, NULL);
	glCompileShader(fragmentShader);

	GLint status;
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &status);
	if(status != GL_TRUE){
		printf("Vertex shader failed to compile\n");

		int maxLength = 2048;
		std::vector<GLchar> errorLog(maxLength);
		glGetShaderInfoLog(vertexShader, maxLength, &maxLength, &errorLog[0]);

		printf("%s\n", (char *) &errorLog[0]);
	}

	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &status);
	if(status != GL_TRUE){
		printf("Fragment shader failed to compile\n");

		int maxLength = 2048;
		std::vector<GLchar> errorLog(maxLength);
		glGetShaderInfoLog(fragmentShader, maxLength, &maxLength, &errorLog[0]);

		printf("%s\n", (char *) &errorLog[0]);

	}

	GLuint shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);

	//glBindFragDataLocation(shaderProgram, 0, "outColor");

	glLinkProgram(shaderProgram);




	// Wrap in object

	Shader *s = new Shader();
	s->program = shaderProgram;
	s->posAttrib = glGetAttribLocation(shaderProgram, "position");
	s->normalAttrib = glGetAttribLocation(shaderProgram, "normal");
	s->colorAttrib = glGetAttribLocation(shaderProgram, "color");
	s->uniModelViewAttrib = glGetUniformLocation(shaderProgram, "modelview");
	s->uniProjAttrib = glGetUniformLocation(shaderProgram, "proj");
	s->eyePosAttrib = glGetUniformLocation(shaderProgram, "eyePosition");

	return s;

}

void Shader::setLights(const vector<LightSource> &lights){

	if(lights.size() > MAXLIGHTS){
		cout << "Too many lights!" << endl;
	}

	GLuint nlightsAttr = glGetAttribLocation(program, "nlights");
	glUniform1i(nlightsAttr, lights.size());


	for(int i = 0; i < lights.size(); i++){

		string si = to_string(i);

		GLuint pAttr = glGetUniformLocation(program, ("lights["+si+"].position").c_str()),
				aAttr = glGetUniformLocation(program, ("lights["+si+"].ambient").c_str()),
				dAttr = glGetUniformLocation(program, ("lights["+si+"].diffuse").c_str()),
				sAttr = glGetUniformLocation(program, ("lights["+si+"].specular").c_str());

		glUniform4fv(pAttr, 1, glm::value_ptr(lights[i].position));
		glUniform3fv(aAttr, 1, glm::value_ptr(lights[i].ambient));
		glUniform3fv(dAttr, 1, glm::value_ptr(lights[i].diffuse));
		glUniform3fv(sAttr, 1, glm::value_ptr(lights[i].specular));
	}


}


void Shader::setMaterial(const Material &m){

	GLuint aAttr = glGetUniformLocation(program, "material.ambient"),
			dAttr = glGetUniformLocation(program, "material.diffuse"),
			sAttr = glGetUniformLocation(program, "material.specular"),
			shAttr = glGetUniformLocation(program, "material.shininess");

	glUniform3fv(aAttr, 1, glm::value_ptr(m.ambient));
	glUniform3fv(dAttr, 1, glm::value_ptr(m.diffuse));
	glUniform3fv(sAttr, 1, glm::value_ptr(m.specular));
	glUniform1f(shAttr, m.shininess);
}




static const char *flat_vertex_glsl =
	#include "shaders/flat.vertex.glsl"
;
static const char *flat_fragment_glsl =
	#include "shaders/flat.fragment.glsl"
;
Shader *Shader::Default(){
	return Shader::Load(
		flat_vertex_glsl,
		flat_fragment_glsl
	);
}


static const char *gouraud_vertex_glsl =
	#include "shaders/gouraud.vertex.glsl"
;
static const char *gouraud_fragment_glsl =
	#include "shaders/gouraud.fragment.glsl"
;
Shader *Shader::Gouraud(){
	return Shader::Load(
		gouraud_vertex_glsl,
		gouraud_fragment_glsl
	);
}


static const char *phong_vertex_glsl =
	#include "shaders/phong.vertex.glsl"
;
static const char *phong_fragment_glsl =
	#include "shaders/phong.fragment.glsl"
;
Shader *Shader::Phong(){
	return Shader::Load(
		phong_vertex_glsl,
		phong_fragment_glsl
	);
}

}
}
