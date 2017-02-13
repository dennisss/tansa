#include <tansa/data.h>

#include <cstdlib>
#include <fstream>

namespace tansa {


DataObject DataObject::LoadFile(std::string filename) {

	std::string extension = filename.substr(filename.rfind('.') + 1);

	// Compile JS files to JSON before loading them
	if(extension == "js") {
		std::string cmd = "scripts/jsdata_driver.js " + filename;
		system(cmd.c_str());
		filename = filename + ".o";
	}

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
