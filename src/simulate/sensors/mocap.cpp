#include <tansa/model.h>

#include <chrono>


namespace tansa {

extern Vector3d noiseVector(std::normal_distribution<> &dist, std::default_random_engine &gen);


Quaterniond noiseQuaternion(std::normal_distribution<> &dist, std::default_random_engine &gen) {
	Vector3d e = Vector3d( dist(gen), dist(gen), dist(gen) );
	return (AngleAxisd(e[0], Vector3d::UnitZ()) * AngleAxisd(e[1], Vector3d::UnitX()) * AngleAxisd(e[2], Vector3d::UnitZ())).matrix();
}

MocapSensor::MocapSensor(const DataObject &desc) {

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	gen = std::default_random_engine(seed);

	rate = 50; //desc["rate"];
	posNoise = std::normal_distribution<>(0, 0.001);
	orientNoise = std::normal_distribution<>(0, 0.0001); // in radians

}

void MocapSensor::update(MocapSensorState &s, const Time &t) {
	s.time = t;
}


void MocapSensor::observe(MocapSensorState &s, const ModelState &ms) {

	double dt = s.time.since(s.lastReading).seconds();

	// TODO: Have separate rates for independent sensors
	if(dt < 1.0 / rate) {
		return;
	}

	MocapSensorData data;
	data.position = ms.position + noiseVector(posNoise, gen);
	data.orientation = noiseQuaternion(orientNoise, gen) * ms.orientation
	data.time = s.time;
	s.lastReading = s.time;
	this->publish(data);
}

}
