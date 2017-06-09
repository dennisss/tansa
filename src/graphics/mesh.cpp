#include <tansa/graphics/mesh.h>
#include "utils.h"

#include <vector>
#include <iostream>

#include <cstring>

using namespace std;

namespace tansa {
namespace graphics {

using namespace glm;

Mesh::Mesh(const vector<vec3> &vertices, const vector<vec3> &colors, vector<vector<unsigned int>> faces, Shader *shader) : Mesh(vertices, faces, shader) {

	// Create color buffer
	glGenBuffers(1, &color_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, color_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * colors.size(), &colors[0], GL_STATIC_DRAW);
	glVertexAttribPointer(shader->colorAttrib, 3, GL_FLOAT, GL_FALSE, 0, 0);

}


Mesh::Mesh(const vector<vec3> &vertices, vector<vector<unsigned int>> faces, Shader *shader) : Object(shader) {
	// Setup shader
	// TODO: Do I need to reset the program in the draw?
	glUseProgram(shader->program);
	// TODO: Setup uniforms



	// Create vertex buffer
	this->vertices = vertices;
	glGenBuffers(1, &pos_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, pos_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(shader->posAttrib, 3, GL_FLOAT, GL_FALSE, 0, 0);



	// Compute general normals
	vector<vec3> normals(vertices.size(), vec3(0,0,0));
	for(vector<unsigned int> &face : faces){
		// TODO: Check this
		vec3 p0 = vertices[face[0]], p1 = vertices[face[1]], p2 = vertices[face[2]];
		vec3 n = cross(p1 - p0, p2 - p0);

//		if(face[0] >= normals.size() || face[1] >= normals.size() || face[2] >= normals.size())
//			cout << "ERR!" << endl;

		normals[face[0]] += n; normals[face[1]] += n; normals[face[2]] += n;
	}
	for(int i = 0; i < normals.size(); i++){
		normals[i] = normalize(normals[i]);
	}

	// Create normal buffer
	glGenBuffers(1, &normal_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, normal_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * normals.size(), &normals[0], GL_STATIC_DRAW);
	glVertexAttribPointer(shader->normalAttrib, 3, GL_FLOAT, GL_FALSE, 0, 0);



	vector<GLuint> indices(3*faces.size());
	for(int i = 0; i < faces.size(); i++){
		if(faces[i].size() != 3)
			cerr << "Only triangle meshes are supported" << endl;

		indices[3*i] = faces[i][0];
		indices[3*i + 1] = faces[i][1];
		indices[3*i + 2] = faces[i][2];
	}

	// Create indice buffer
	this->nindices = indices.size();
	glGenBuffers(1, &index_vbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * indices.size(), &indices[0], GL_STATIC_DRAW);



}


Mesh::~Mesh() {
	// Destroy VBOs
}



void Mesh::draw(Camera *cam, Transform *modelview){
	Object::draw(cam, modelview);
	glDrawElements(GL_TRIANGLES, nindices, GL_UNSIGNED_INT, NULL);
}


Mesh *Mesh::read(const char *filename, Shader *shader){


	const char *ext = get_filename_ext(filename);


	if(strcmp(ext, "smf") == 0){
		return read_smf(filename, shader);
	}
	else if(strcmp(ext, "stl") == 0) {
		return read_stl(filename, shader);
	}
//	else if(strcmp(ext, "txt") == 0){
//		return read_patch(filename, shader);
//	}


	cerr << "Unknown mesh format: " << ext << endl;
	return NULL;
}


}
}
