#include <tansa/data.h>

#include <iostream>
#include <cstdlib>
#include <fstream>

namespace tansa {


DataObject DataObject::LoadFile(std::string filename) {

	std::string extension = filename.substr(filename.rfind('.') + 1);

	// Compile JS files to JSON before loading them
	if(extension == "js") {
		std::string cmd = "scripts/jsdata_driver.js " + filename;

		if(system(cmd.c_str()) != 0) {
			throw std::runtime_error("Failed to execute javascript driver");
		}

		filename = filename + ".o";
	}

	std::ifstream stream(filename);
	if(!stream.is_open()){
		throw std::runtime_error("Failed to open data file: " + filename);
	}


	std::string rawdata((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
	json data = json::parse(rawdata);

	// JS driver
	if(data.count("error") == 1) {
		std::cout << data["error"] << std::endl;

		throw std::runtime_error("Failed to parse file: " + filename);
	}

	return DataObject(data);

}







}
