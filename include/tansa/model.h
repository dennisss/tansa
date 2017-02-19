#ifndef TANSA_MODEL_H
#define TANSA_MODEL_H

#include "core.h"
#include "time.h"
#include "channel.h"

#include <Eigen/Dense>
#include <memory>
#include <random>

using namespace Eigen;


namespace tansa {


#define GRAVITY_MS 9.8


/**
 * Base class for all types of state data
 */
struct State {

	typedef std::shared_ptr<State> Ptr;

	Time time = Time(0, 0);

};



struct ModelState : State {

	typedef std::shared_ptr<ModelState> Ptr;

	// State variables for rigid body dynamics
	Vector3d position = Vector3d(0,0,0);
	Vector3d velocity = Vector3d(0,0,0);
	Vector3d acceleration = Vector3d(0,0,0);

	Quaterniond orientation = Quaterniond(1, 0, 0, 0);
	Vector3d angularVelocity = Vector3d(0,0,0);
	Vector3d angularAcceleration = Vector3d(0,0,0);
};

/**
 * A generic entity representing a rigid body system
 */
class Model {
public:

	typedef std::shared_ptr<Model> Ptr;

	Model(const DataObject &desc);

	virtual State::Ptr defaultState();

	virtual void update(State::Ptr s, const Time &t);

	virtual Vector3d force(const State &s) = 0;
	virtual Vector3d torque(const State &s) = 0;


private:

	double mass;
	Matrix3d mInertia;

};

/**
 * An entity like a sensor which can monitor the state of the system on every
 */
class ModelObserver {

public:



};




struct BatteryState : State {
	/** How much of the power is remaining */
	double percent;

};

// TODO: This should also be a sensor? (or integrate )
class Battery {
public:

	Battery(const DataObject &desc);


	void update(BatteryState &s, const Time &t);

	double voltage(const BatteryState &s);


};


// Rotations based on right hand rule
#define COUNTERCLOCKWISE 1
#define CLOCKWISE -1



struct MotorState : State {
	//double speed;
	//double commandedSpeed;

	double throttle = 0;
	double commandedThrottle = 0;
};

/**
 * For simulating a brushless three-phase motor, ESC, and fixed pitch propeller
 */
class Motor {
public:

	Motor(const DataObject &desc);

	void control(MotorState &s, double throttle);
	void update(MotorState &s, const Time &t);

	Vector3d force(const MotorState &s);
	Vector3d torque(const MotorState &s);


	Vector3d position;
	double spin; // Clockwise (-1) or counter-clockwise (+1)
	double thrustCoeff;
	double torqueCoeff;

	double timeConstantUp;
	double timeConstantDown;
};



struct IMUState : State {
	Vector3d gyroBias = Vector3d(0, 0, 0);

	Time lastReading;
};

struct IMUSensorData {
	static const int ID = 1;

	Time time;
	Vector3d accel;
	Vector3d gyro;
	Vector3d mag;
};


class IMU : public Channel {
public:

	IMU(const DataObject &desc);


	void update(IMUState &s, const Time &t);

	void observe(IMUState &s, const ModelState &ms);


private:

	std::default_random_engine gen;

	double accelRate;
	std::normal_distribution<> accelNoise;

	double gyroRate;
	std::normal_distribution<> gyroNoise;
	std::normal_distribution<> gyroBiasWalkNoise;

	std::normal_distribution<> magNoise;
};


struct GPSState : State {
	Vector3d gyroBias = Vector3d(0, 0, 0);

	Time lastReading;
};

struct GPSData {
	static const int ID = 1;

	Time time;
	Vector3d latLongAlt;
	Vector3d vel;
};

class GPS : public Channel {

public:

	GPS(const DataObject &desc);

	void update(GPSState &s, const Time &t);

	void observe(GPSState &s, const ModelState &ms);

private:
	std::default_random_engine gen;

	double rate;
	std::normal_distribution<> noise;

};



struct MultirotorModelState : ModelState {
	// Multirotor specific stuff
	BatteryState battery;
	std::vector<MotorState> motors;

	IMUState imu;
	GPSState gps;
};

/**
 * Customizable multi-rotor model
 */
class MultirotorModel : public Model {
public:

	typedef std::shared_ptr<MultirotorModel> Ptr;


	/**
	 * Creates a new model from a model file
	 */
	MultirotorModel(const DataObject &desc);
	virtual ~MultirotorModel();

	virtual State::Ptr defaultState();


	virtual void update(State::Ptr s, const Time &t);

	// Sum up motor thrusts
	virtual Vector3d force(const State &s);

	// Sum up motor torques
	virtual Vector3d torque(const State &s);


	// Various sub-components of the multi-rotor
	std::vector<Motor *> motors;
	IMU *imu;
	GPS *gps;
	Battery *battery;

};



}


#endif
