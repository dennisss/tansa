#ifndef TANSA_ESTIMATION_H
#define TANSA_ESTIMATION_H

#include "time.h"
#include "model.h"


namespace tansa {

typedef Vector3d ControlInput;


class StateEstimator {



};

class LinearComplementaryEstimator {
public:
	/**
	 * Forward predict state upto a time given the control input being applied
	 */
	void predict(ModelState &s, ControlInput u, const Time &t);


	/**
	 * Update the state given an observation at a certain time
	 */
	void correct(ModelState &s, const Vector3d &x, const Vector3d &v, const Time &t);


};

}

#endif
