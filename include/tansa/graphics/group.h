#ifndef TANSA_GRAPHICS_GROUP_H_
#define TANSA_GRAPHICS_GROUP_H_

#include "drawable.h"

#include <vector>

namespace tansa {
namespace graphics {

/**
 * Many shapes grouped together and drawn/transformed together
 */
class Group : public Drawable {

public:

	Group();

	void draw(Camera *cam, Transform *modelview);

	void addObject(Drawable *o);

	void removeObject(Drawable *o);

private:

	std::vector<Drawable *> objects;

};


}
}


#endif
