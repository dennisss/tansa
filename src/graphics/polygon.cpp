#include <tansa/graphics/polygon.h>

#include <cmath>

using namespace std;

namespace tansa {
namespace graphics {

using namespace glm;

static vector<vec3> regular_polygon(int nsides){

	vector<vec3> pts;

	float dtheta = 2.0*M_PI / nsides;
	float theta = -(M_PI/2.0) + (dtheta / 2.0);

	for(int i = 0; i < nsides; i++){
		pts.push_back(vec3(cos(theta), sin(theta), 0));
		theta += dtheta;
	}


	return pts;
}





Polygon::Polygon(vector<vec3> verts, vector<vec3> colors, Shader *shader) : Object(shader) {

	this->nvertices = verts.size();

	// Create vertex buffer
	glGenBuffers(1, &pos_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, pos_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * verts.size(), &verts[0], GL_STATIC_DRAW);
	glVertexAttribPointer(shader->posAttrib, 3, GL_FLOAT, GL_FALSE, 0, 0);

	// Create color buffer
	glGenBuffers(1, &color_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, color_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * colors.size(), &colors[0], GL_STATIC_DRAW);
	glVertexAttribPointer(shader->colorAttrib, 3, GL_FLOAT, GL_FALSE, 0, 0);
}


void Polygon::setColor(vec3 color){

	vector<vec3> colors(this->nvertices, color);

	glBindBuffer(GL_ARRAY_BUFFER, color_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * colors.size(), &colors[0], GL_STATIC_DRAW);
}


void Polygon::draw(Camera *cam, Transform *modelview){
	Object::draw(cam, modelview);
	glDrawArrays(GL_TRIANGLE_FAN, 0, nvertices);
}


Polygon *Polygon::Regular(int nsides, vec3 color, Shader *s){

	vector<vec3> colors(nsides);
	for(int i = 0; i < colors.size(); i++){
		colors[i] = color;
	}

	return Polygon::Regular(nsides, colors, s);

}

Polygon *Polygon::Regular(int nsides, vector<vec3> colors, Shader *s){
	return new Polygon(regular_polygon(nsides), colors, s);
}


}
}
