#ifndef TANSA_PID_H_
#define TANSA_PID_H_

#include <Eigen/Dense>

using namespace Eigen;

// n-dimensional PID filter

class PID{

public:
    PID();

    /**
	 * Compute output of filter given the state error and change in time
	 */
    Vector3d compute(Vector3d e, double dt);

	/**
	 * Like the regular compute but computes given a known error derivative
	 */
	Vector3d compute(Vector3d e, Vector3d de, double dt);


    void setGains(Vector3d gP, Vector3d gI, Vector3d gD);

 //   void set

    void setLimit(Vector3d min, Vector3d max);



private:

    Vector3d gainP;
    Vector3d gainI;
    Vector3d gainD;

    Vector3d lastE;
    Vector3d sumE;

	Vector3d minIntegral;
	Vector3d maxIntegral;

};




#endif
