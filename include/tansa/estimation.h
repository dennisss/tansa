#ifndef TANSA_ESTIMATION_H_
#define TANSA_ESTIMATION_H_

#include "time.h"

#include <Eigen/Dense>


using namespace Eigen;

typedef Vector3d ControlInput;

struct State {
	State() : time(0,0) {};


	Vector3d position;
	Vector3d velocity;
	Quaterniond orientation;

	/* When this state is valid */
	Time time;
};


class StateEstimator {



};

class LinearComplementaryEstimator {
public:
	/**
	 * Forward predict state upto a time given the control input being applied
	 */
	void predict(State &s, ControlInput u, const Time &t);


	/**
	 * Update the state given an observation at a certain time
	 */
	void correct(State &s, const Vector3d &x, const Vector3d &v, const Time &t);


};



#endif
