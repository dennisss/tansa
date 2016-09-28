#ifndef TANSA_PID_H_
#define TANSA_PID_H_

#include <Eigen/Dense>

using namespace Eigen;

/**
 * n-dimensional PID filter
 */
template <unsigned int N>
class PID{
public:

	typedef Vector<N, double> Vector;

    PID();

    /**
	 * Compute output of filter given the state error and change in time
	 */
    Vector &compute(Vector e, double dt);

	/**
	 * Like the regular compute but computes given a known error derivative
	 */
	Vector &compute(Vector e, Vector de, double dt);


    void setGains(Vector gP, Vector gI, Vector gD);



	void setOutputLimits(Vector min, Vector max);

	/**
	 * Sets the maximum control effect produced by the integral
	 * This should be called after setting the gains
	 * Note: this is not the value of the error integral, but the output when gain is applied
	 */
    void setWindupOutputLimit(Vector min, Vector max);



private:

    Vector gainP;
    Vector gainI;
    Vector gainD;

    Vector lastE;
    Vector sumE;

	Vector minIntegral;
	Vector maxIntegral;

};




#endif
