#include <tansa/data.h>

#include <fstream>

namespace tansa {


DataObject DataObject::LoadFile(std::string filename) {

	std::ifstream stream(filename);
	if(!stream) {
		// TODO: Return failure
		//return false;
	}

	std::string rawdata((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
	json data = json::parse(rawdata);

	return DataObject(data);

}







}
