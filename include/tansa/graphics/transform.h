#ifndef TANSA_GRAPHICS_TRANSFORM_H_
#define TANSA_GRAPHICS_TRANSFORM_H_

#include "glm.h"
#include "lighting.h"

#include <vector>


namespace tansa {
namespace graphics {


// Compute a transformation that normalizes (centers and scales to -1 to 1) the points
glm::mat4 Normalizing(std::vector<glm::vec3> &pts);


class Transform {

public:

	Transform();
	Transform(glm::mat4 m);

	virtual glm::mat4 matrix();

	Transform apply(glm::mat4 rhs);


private:
	glm::mat4 trans;

};


class Camera : public Transform {

public:

	Camera();

	glm::mat4 matrix();


	std::vector<LightSource> lights;

	glm::mat4 view;
	glm::mat4 proj;

	glm::vec3 position;

	//inline void set(glm::mat4 m){ proj = m; };


private:



};


}
}

#endif
