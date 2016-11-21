#include <tansa/vehicle.h>
#include <tansa/core.h>

#include <fstream>

void Vehicle::readParams(string file) {

	std::ifstream stream(file);
	if(!stream) throw "Unable to read params file!";

	std::string rawdata((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
	nlohmann::json data = nlohmann::json::parse(rawdata);

	if(data.count("gains")) {
		params.gains.p = Point(data["gains"]["p"][0], data["gains"]["p"][1], data["gains"]["p"][2]);
		params.gains.i = Point(data["gains"]["i"][0], data["gains"]["i"][1], data["gains"]["i"][2]);
		params.gains.d = Point(data["gains"]["d"][0], data["gains"]["d"][1], data["gains"]["d"][2]);
	}

	if(data.count("hoverPoint")) {
		params.hoverPoint = data["hoverPoint"];
	}


	if(data.count("latency")) {
		params.latency = data["latency"];
	}

}
