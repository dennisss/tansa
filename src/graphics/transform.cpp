#include <tansa/graphics/transform.h>

#include <cfloat>

namespace tansa {
namespace graphics {

Transform::Transform(){
	trans = glm::mat4(1.0);
}

Transform::Transform(glm::mat4 m){
	trans = m;
}


glm::mat4 Transform::matrix(){
	return trans;
}

Transform Transform::apply(glm::mat4 rhs){
	return Transform(trans * rhs);
}



Camera::Camera() : Transform() {
	proj = glm::mat4(1.0f);
}


glm::mat4 Camera::matrix(){
	return proj * Transform::matrix();
}



glm::mat4 Normalizing(std::vector<glm::vec3> &pts){

	float xmin = FLT_MAX, xmax = FLT_MIN,
		  ymin = FLT_MAX, ymax = FLT_MIN,
		  zmin = FLT_MAX, zmax = FLT_MIN;

	// Compute AABB
	for(const glm::vec3 &v : pts){
		if(v.x < xmin)
			xmin = v.x;
		else if(v.x > xmax)
			xmax = v.x;

		if(v.y < ymin)
			ymin = v.y;
		else if(v.y > ymax)
			ymax = v.y;

		if(v.z < zmin)
			zmin = v.z;
		else if(v.z > zmax)
			zmax = v.z;
	}


	return glm::scale(glm::vec3(
		2.0f / (xmax - xmin),
		2.0f / (ymax - ymin),
		2.0f / (zmax - zmin)
	)) * glm::translate(glm::vec3(
		-(xmax + xmin) / 2.0f,
		-(ymax + ymin) / 2.0f,
		-(zmax + zmin) / 2.0f
	));
}


}
}
