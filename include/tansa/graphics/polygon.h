#ifndef TANSA_GRAPHICS_POLYGON_H_
#define TANSA_GRAPHICS_POLYGON_H_

#include "drawable.h"

#include <vector>

namespace tansa {
namespace graphics {


/* Convex polygon drawing */
class Polygon : public Object {

public:

	Polygon(std::vector<glm::vec3> verts, std::vector<glm::vec3> colors, Shader *s);


	static Polygon *Regular(int nsides, std::vector<glm::vec3> colors, Shader *s);
	static Polygon *Regular(int nsides, glm::vec3 color, Shader *s);


	// Changes the color of all vertices to one color
	void setColor(glm::vec3 color);


	void draw(Camera *cam, Transform *modelview);


private:

	GLuint pos_vbo, color_vbo;

	int nvertices;

};

}
}

#endif
