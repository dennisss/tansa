#include <tansa/graphics/group.h>

namespace tansa {
namespace graphics {

Group::Group() : Drawable(){

}

void Group::draw(Camera *cam, Transform *prev){

	Transform t = prev->apply(this->trans);

	for(int i = 0; i < objects.size(); i++){
		objects[i]->draw(cam, &t);
	}
}


void Group::addObject(Drawable *o){
	objects.push_back(o);
}


void Group::removeObject(Drawable *o){
	for(int i = 0; i < objects.size(); i++){
		if(objects[i] == o){
			objects.erase(objects.begin() + i);
			return;
		}
	}
}

}
}
