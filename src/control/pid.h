#ifndef TANSA_PID_H_
#define TANSA_PID_H_

#include <Eigen/Dense>
#include <float.h>

#include <memory>


namespace tansa {

/**
 * n-dimensional PID filter
 */
template <unsigned int N>
class PID {
public:

	typedef Eigen::Matrix<double, N, 1> Vector;


	/**
	 * All time varying parts of the PID filter
	 */
	struct State {
		typedef std::shared_ptr<State> Ptr;

		Vector lastE;
		Vector sumE;
	};

    PID(std::shared_ptr<State> state) {
		this->state = state;
		state->sumE = Vector::Zero();
		state->lastE = Vector::Zero();

		for(unsigned i = 0; i < N; i++) {
			minIntegral(i) = -DBL_MAX;
			maxIntegral(i) = DBL_MAX;
		}
	}

    /**
	 * Compute output of filter given the state error and change in time
	 */
    inline Vector compute(Vector e, double dt) {
		Vector de = (e - state->lastE) / dt;
		state->lastE = e;

		return compute(e, de, dt);
	}

	/**
	 * Like the regular compute but computes given a known error derivative
	 */
	inline Vector compute(Vector e, Vector de, double dt) {
		// Update integral
		// We multiply by the integral gain here so that the sum value is in units of acceleration
		state->sumE += gainI.cwiseProduct(dt*e);

		// Check limits
		for(unsigned int i = 0; i < N; i++) {
			if(state->sumE(i) > maxIntegral(i))
				state->sumE(i) = maxIntegral(i);
			else if(state->sumE(i) < minIntegral(i))
				state->sumE(i) = minIntegral(i);
		}

		// Compute control output
		Vector out = gainP.cwiseProduct(e) + state->sumE + gainD.cwiseProduct(de);

		// TODO: Contrain to output limits

		return out;
	}


    void setGains(Vector gP, Vector gI, Vector gD) {
		this->gainP = gP;
		this->gainI = gI;
		this->gainD = gD;
	}



	void setOutputLimits(Vector min, Vector max);

	/**
	 * Sets the maximum control effect produced by the integral
	 * This should be called after setting the gains
	 * Note: this is not the value of the error sum, but the integral output component after gain is applied
	 */
    void setWindupOutputLimit(Vector min, Vector max) {
		this->minIntegral = min;
		this->maxIntegral = max;
	}



private:
	std::shared_ptr<State> state;

    Vector gainP;
    Vector gainI;
    Vector gainD;

	Vector minIntegral;
	Vector maxIntegral;
};


}

#endif
