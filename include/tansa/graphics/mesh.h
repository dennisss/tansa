#ifndef TANSA_GRAPHICS_MESH_H_
#define TANSA_GRAPHICS_MESH_H_

#include "drawable.h"
#include "shader.h"

#include <vector>

namespace tansa {
namespace graphics {

/**
 * Drawing of generic triangle based meshes
 */
class Mesh : public Object {

public:

	// TODO: Also accept an array of normals

	Mesh(const std::vector<glm::vec3> &vertices, const std::vector<glm::vec3> &colors, std::vector<std::vector<unsigned int> > faces, Shader *shader);

	// For colorless meshes using a material
	Mesh(const std::vector<glm::vec3> &vertices, std::vector<std::vector<unsigned int> > faces, Shader *shader);

	~Mesh();

	// TODO: Generalize this
	static Mesh *read(const char *filename, Shader *shader);


	void draw(Camera *cam, Transform *modelview);


	std::vector<glm::vec3> vertices;

private:


	GLuint pos_vbo, normal_vbo, color_vbo, index_vbo;

	int nindices;


	static Mesh *read_smf(const char *filename, Shader *shader);
	static Mesh *read_stl(const char *filename, Shader *shader);

};




/* A Bezier Patch : Consists of a set of control points, and holds a reference to an interpolated mesh */
class Patch : public Object {

public:

	static Patch *read(char *filename, Shader *shader);


	// control_points should be of size 16 representing a 4*4 grid
	Patch(std::vector<glm::vec3> &control_points, Shader *shader);
	~Patch();

	void draw(Camera *cam, Transform *modelview);


	void generate(int res);

	// The points backing it
	std::vector<glm::vec3> control_points;

private:

	// The currently generated mesh
	Mesh *mesh;
	Shader *shader;
};


}
}

#endif
