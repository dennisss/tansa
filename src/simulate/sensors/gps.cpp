#include <tansa/model.h>

#include <math.h>

#include <chrono>


namespace tansa {

const double homeLatitude = 39.953017;
const double homeLongitude = -75.182947;


const double earthRadius = 6378000; // In meters

extern Vector3d noiseVector(std::normal_distribution<> &dist, std::default_random_engine &gen);

Vector3d latLongAlt(Vector3d localPos) {

	return Vector3d(
		homeLatitude + (localPos.y() / earthRadius) * (180.0 / M_PI),
		homeLongitude + (localPos.x() / earthRadius) * (180.0 / M_PI) / cos(homeLatitude * M_PI / 180),
		localPos.z()
	);

}

GPS::GPS(const DataObject &desc) {

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	gen = std::default_random_engine(seed);

	rate = 10; //desc["rate"];
	noise = std::normal_distribution<>(0, 0.1);

}

void GPS::update(GPSState &s, const Time &t) {
	s.time = t;
}


void GPS::observe(GPSState &s, const ModelState &ms) {

	double dt = s.time.since(s.lastReading).seconds();

	// TODO: Have separate rates for independent sensors
	if(dt < 1.0 / rate) {
		return;
	}

	GPSData data;
	data.latLongAlt = latLongAlt(ms.position + noiseVector(noise, gen));
	data.vel = ms.velocity + noiseVector(noise, gen);
	data.time = s.time;
	s.lastReading = s.time;
	this->publish(data);
}

}
