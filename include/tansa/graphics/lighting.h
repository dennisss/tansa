#ifndef TANSA_GRAPHICS_LIGHTING_H_
#define TANSA_GRAPHICS_LIGHTING_H_

#include "glm.h"

namespace tansa {
namespace graphics {

// These should match the shaders

#define MAXLIGHTS 4

struct LightSource {
	glm::vec4 position;

	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;

};

struct Material {

	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;
	GLfloat shininess;

};

}
}

#endif
