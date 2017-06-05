#ifndef TANSA_GRAPHICS_SHADER_H_
#define TANSA_GRAPHICS_SHADER_H_

#include "glm.h"

#include "lighting.h"

#include <fstream>
#include <vector>


namespace tansa {
namespace graphics {

class Shader {


public:

	/* Initialize the shader for the current vertex array object */
	void init();


	static Shader *Load(std::ifstream &&vertexFile, std::ifstream &&fragmentFile);
	static Shader *Load(const char *vertexSrc, const char *fragmentSrc);

	static Shader *Default();
	static Shader *Gouraud();
	static Shader *Phong();





	void setLights(const std::vector<LightSource> &lights);
	void setMaterial(const Material &m);


	GLuint program;
	GLuint posAttrib;
	GLuint normalAttrib;
	GLuint colorAttrib;

	GLuint uniProjAttrib, uniModelViewAttrib;

	GLuint eyePosAttrib;


};


}
}

#endif
