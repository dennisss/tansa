#ifndef TANSA_GRAPHICS_DRAWABLE_H_
#define TANSA_GRAPHICS_DRAWABLE_H_

#include "shader.h"
#include "transform.h"
#include "glm.h"


namespace tansa {
namespace graphics {

/*
	An object than can be drawn. This class handles configuring transforms, viewports, and projection

*/
class Drawable {

public:

	Drawable();
	virtual ~Drawable();

	// Draw the object. 'proj' contains all transformations that should be applied to it
	virtual void draw(Camera *cam, Transform *modelview) = 0;


	//
	void transform(glm::mat4 m);
	void apply(glm::mat4 m);


protected:

	glm::mat4 trans;


};


/*
	Every object has its own vertex array object
*/
class Object : public Drawable {

public:

	Object(Shader *shader);
	virtual ~Object();


	virtual void draw(Camera *cam, Transform *modelview);


	void setMaterial(Material *m);

	// Works for shaders when the current shader shares the exact same attributes (and ordering of them) as the new shader
	inline void setShader(Shader *s){ this->shader = s; };

protected:

	Shader *shader;


	// Optional material
	Material *material;

private:
	GLuint vao;

};

}
}

#endif
