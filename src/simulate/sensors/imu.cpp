#include <tansa/model.h>

#include <random>
#include <chrono>

/*

Given the simulation state, this should


Parameters:
- Noise variances on each one
- Bias random walk parameters


The imu gyro has a state characterized by the current bias

*/

namespace tansa {

IMU::IMU(const DataObject &desc) {

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	gen = std::default_random_engine(seed);

	accelRate = desc["accel_rate"];
	accelNoise = std::normal_distribution<>(0, desc["accel_noise"]);

	gyroRate = desc["gyro_rate"];
	gyroNoise = std::normal_distribution<>(0, desc["gyro_noise"]);
	gyroBiasWalkNoise = std::normal_distribution<>(0, desc["gyro_random_walk_noise"]);

	magNoise = std::normal_distribution<>(0, 0.05);

}


Vector3d noiseVector(std::normal_distribution<> &dist, std::default_random_engine &gen) {
	return Vector3d(dist(gen), dist(gen), dist(gen));
}


void IMU::update(IMUState &s, const Time &t) {
	// Integrate forward biases here
	double dt = t.since(s.time).seconds();
	s.gyroBias += dt * noiseVector(gyroBiasWalkNoise, gen); // TODO: Check this over
	s.time = t;
}


void IMU::observe(IMUState &s, const ModelState &ms) {

	double dt = s.time.since(s.lastReading).seconds();

	// TODO: Have separate rates for independent sensors
	if(dt < 1.0 / accelRate) {
		return;
	}

	Matrix3d Ri = ms.orientation.matrix().transpose();

	// TODO: These all need to be rotated into the body frame
	// TODO: Add gravity to the acceleration
	Vector3d a = Ri * (ms.acceleration + Vector3d(0, 0, GRAVITY_MS)) + noiseVector(accelNoise, gen);
	Vector3d g = ms.angularVelocity + noiseVector(gyroNoise, gen); // + s.gyroBias;
	Vector3d m = Ri * Vector3d(0, 0.65, 0) + noiseVector(magNoise, gen);

	IMUSensorData data;
	data.accel = a;
	data.gyro = g;
	data.mag = m;
	data.time = s.time;
	s.lastReading = s.time;
	this->publish(data);
}


}
