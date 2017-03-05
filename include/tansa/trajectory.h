#ifndef TANSA_TRAJECTORY_H_
#define TANSA_TRAJECTORY_H_

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <initializer_list>

using namespace Eigen;
using namespace std;


namespace tansa {

const unsigned PointDims = 3;
// TODO: Should eventually be Vector4d to incorporate yaw
typedef Vector3d Point;

enum PointDerivative {
	PointPosition = 0,
	PointVelocity = 1,
	PointAcceleration = 2,
	PointJerk = 3,
	PointSnap = 4,
	PointMaxOrder = 5
};


/**
 * A Point with one or more Points constraining the derivates at that point
 */
class ConstrainedPoint {
public:

	/**
	 * Creates a constrained point which is not moving
	 */
	static ConstrainedPoint Stationary(const Point &p) {
		ConstrainedPoint cp;
		cp.constrain((PointDerivative) 0, p);
		for(int i = 1; i < PointMaxOrder; i++) {
			cp.constrain((PointDerivative) i, Point::Zero());
		}

		return cp;
	}

	ConstrainedPoint() {
		for(int i = 0; i < PointMaxOrder; i++)
			constrained[i] = false;
	}

	ConstrainedPoint(std::initializer_list<Point> l) {
		auto it = l.begin();
		for(int i = 0; i < PointMaxOrder; i++) {
			if(it != l.end()) {
				//cout << *it << endl;
				constrain((PointDerivative) i, *it);
				it++;
			}
			else {
				constrained[i] = false;
			}
		}
    }

	void constrain(PointDerivative order, const Point &p) {
		points[order] = p;
		constrained[order] = true;
	}

	bool isConstrained(unsigned order) const {
		return constrained[order];
	}

	Point &operator[](std::size_t idx) {
		return points[idx];
	}

	const Point &operator[](std::size_t idx) const {
		return points[idx];
	}

private:
	// We currently support constraining at most position, velocity, and acceleration
	Point points[PointMaxOrder];
	bool constrained[PointMaxOrder];

};


// Used for determining feasibility of trajectories
#define MAX_ACCELERATION 3


/**
 * Evaluation of a trajectory at a point in time
 */
struct TrajectoryState {
	Point position;
	Point velocity;
	Point acceleration;
};


/**
 * A path that the vehicle should follow constrained to a given time period
 * They are parametrized w.r.t. time
 * Should be at least a three time differentiable function
 */
class Trajectory {
public:

	typedef shared_ptr<Trajectory> Ptr;

	Trajectory(double t1, double t2) {
		this->t1 = t1;
		this->t2 = t2;
	}
	virtual ~Trajectory() {};
	virtual TrajectoryState evaluate(double t) = 0;

	inline double startTime() { return this->t1; }
	inline double endTime() { return this->t2; }

protected:

	double t1, t2;

};


class CompoundTrajectory : public Trajectory {
public:

	CompoundTrajectory(Trajectory::Ptr x1, Trajectory::Ptr x2, double t1, double t2) : Trajectory(t1, t2) {
		this->x1 = x1;
		this->x2 = x2;
	}

	virtual ~CompoundTrajectory() {}

	virtual TrajectoryState evaluate(double t) {

		TrajectoryState s1 = x1->evaluate(t), s2 = x2->evaluate(t);
		s1.position += s2.position;
		s1.velocity += s2.velocity;
		s1.acceleration += s2.acceleration;

		return s1;
	}

private:
	Trajectory::Ptr x1;
	Trajectory::Ptr x2;

};

class TransformedTrajectory : public Trajectory {
public:

	TransformedTrajectory(Trajectory::Ptr x, Matrix3d m, Vector3d p, double t1, double t2) : Trajectory(t1, t2) {
		this->x = x;
		this->m = m;
		this->p = p;
	}

	virtual ~TransformedTrajectory() {}

	virtual TrajectoryState evaluate(double t) {

		TrajectoryState s = x->evaluate(t);
		s.position = m*s.position + p;
		s.velocity = m*s.velocity;
		s.acceleration = m*s.acceleration;

		return s;
	}

private:
	Trajectory::Ptr x;
	Matrix3d m;
	Vector3d p;

};

/**
 * Concatenation of many temporally offset trajectories
 */
class PiecewiseTrajectory : public Trajectory {
public:
	PiecewiseTrajectory(const vector<Trajectory::Ptr> &trajs, double ts, double te) : Trajectory(ts, te) {
		double tot = 0;
		for(auto tr : trajs) {
			tot += tr->endTime() - tr->startTime();
		}

		// TODO: Assert tot = ts - te

		this->trajs = trajs;
	}

	virtual ~PiecewiseTrajectory() {}


	virtual TrajectoryState evaluate(double t) {
		double ts = this->startTime();
		int i = 0;
		for(i = 0; i < trajs.size(); i++) {
			double dur = trajs[i]->endTime() - trajs[i]->startTime();

			if(t >= ts && t <= dur + ts) {
				break;
			}

			ts += dur;
		}

		if(i >= trajs.size()) {
			i = trajs.size() - 1;
			t = this->endTime();
		}

		return trajs[i]->evaluate(t - ts);
	}



private:
	vector<Trajectory::Ptr> trajs;

};

class PolynomialTrajectory : public Trajectory {
public:

	PolynomialTrajectory(const VectorXd c[], double t1, double t2);
	virtual ~PolynomialTrajectory() {}

	/**
	 * Computes an 'optimal' polynomial trajectory between two times given some constraints on the derivatives of the start and end points
	 */
	static PolynomialTrajectory::Ptr compute(const vector<Point> &c1, double t1, const vector<Point> &c2, double t2);


	virtual TrajectoryState evaluate(double t);

private:

	// Store coefficients for x, y, z
	VectorXd coeffs[PointDims];
};


/**
 * Generates a piecewise polynomial trajectory with minimal snap through the given waypoints at the given times
 *
 * Based on 'Minimum Snap Trajectory Generation and Control for Quadrotors' by Mellinger and Kumar
 *
 * See 'Polynomial Trajectory Planning for Quadrotor Flight' by Richter et. al. for a full derivation.
 * This is the constrained QP formulation
 *
 * @param x waypoints through which the trajectory will go
 * @param t time for each waypoint
 * @param corridors if specified, this is the maximum distance away from the straight line between points i and i+1. only applies if the value is > 0. use {} if no corridors are needed
 * @param out where to put the output trajectory
 * @param cost where to put the output cost of the trajectory
 * @return whether or not the optimization succeded
 */
bool compute_minsnap_mellinger11(const vector<ConstrainedPoint> &x, const vector<double> &t, const vector<double> &corridors, Trajectory::Ptr *out, double *cost);

// TODO: Maybe allow ConstrainedPoints to be time constrained and then we can create trajectories with an arbitrary number of constrained times (at least one at beginning and at the end)
/**
 * Similar to compute_minsnap_mellinger11, but takes a total time argument and computes the best possible time intervals for the other segments
 *
 * Optimization implemented using backtracking linear search as per the paper
 */
bool compute_minsnap_optimal_mellinger11(const vector<ConstrainedPoint> &x, double ts, double te, vector<double> corridors, Trajectory::Ptr *out);


/**
 * Smoothly goes in a straight line through two points
 */
class LinearTrajectory : public Trajectory {
public:

	LinearTrajectory(Point x1, double t1, Point x2, double t2);
	virtual ~LinearTrajectory() {};

	virtual TrajectoryState evaluate(double t);

private:

	PolynomialTrajectory::Ptr inner;


};

/**
 * A 2d ellipse with radii along the two major XY axes.
 */
class EllipseTrajectory : public Trajectory {
public:
	EllipseTrajectory(const Point &origin, double radius_x, double radius_y, double theta1, double t1, double theta2, double t2);
	virtual ~EllipseTrajectory(){}
	virtual TrajectoryState evaluate(double t);


private:
	Point origin;
	double radius_x;
	double radius_y;
	double theta1;
	double dtheta;

};

/**
 * A 2d circle in the XY plane. An angle of 0 is on the X axis
 */
class CircleTrajectory : public EllipseTrajectory {
public:
	CircleTrajectory(const Point &origin, double radius, double theta1, double t1, double theta2, double t2)
		: EllipseTrajectory(origin, radius, radius, theta1, t1, theta2, t2) {}
};

/**
 * Stays at one point. Mainly just for testing.
 */
class PointTrajectory : public Trajectory {
public:
	PointTrajectory(const Point &p);
	virtual ~PointTrajectory(){}
	virtual TrajectoryState evaluate(double t);

private:
	Point p;

};

/**
 * Smoothly increases or decreases list intensity between two intensities
 */
class LightTrajectory {
public:

	inline LightTrajectory(double si, double st, double ei, double et) :
			startIntensity(si), startTime(st), endIntensity(ei), endTime(et) {}
	virtual ~LightTrajectory() {}

	// Gives the intensity at a given time between the start and end times
	virtual double evaluate(double t);

	inline double getStartIntensity() { return this->startIntensity; }
	inline double getStartTime() { return this->startTime; }
	inline double getEndIntensity() { return this->endIntensity; }
	inline double getEndTime() { return this->endTime; }


protected:

	double startIntensity, startTime, endIntensity, endTime;
};

/**
 * Smoothly increases or decreases list intensity between two intensities
 */
class StrobeTrajectory : public LightTrajectory {
public:

	inline StrobeTrajectory(double si, double st, double ei, double et, double bps) :
			LightTrajectory(si,st,ei,et), beatsPerSecond(bps) {}
	virtual ~StrobeTrajectory() {}

	// Gives the intensity at a given time between the start and end times
	virtual double evaluate(double t);

	inline double getBeatsPerSecond() { return this->beatsPerSecond; }

private:

	double beatsPerSecond;
};


/*
	Given a circle trajectory between two
*/
class GradualCircleTrajectory : public Trajectory {
public:

	// theta1 and theta2 should also be provided as vectors of position, velocity, acceleration
	GradualCircleTrajectory(const Point &origin, const vector<Point> &c1, double theta1, double t1, const vector<Point> &c2, double theta2, double t2);

	virtual ~GradualCircleTrajectory();

	virtual TrajectoryState evaluate(double t);

private:
	PolynomialTrajectory::Ptr poly;
	CircleTrajectory *circle;
};

/**
 * An Archimedean spiral
 */
class SpiralTrajectory : public Trajectory {
public:

	SpiralTrajectory(const Point &origin, double radius, double theta1, double ts, double theta2, double te);
	virtual ~SpiralTrajectory() {};


	virtual TrajectoryState evaluate(double t);


private:

	Point origin;
	double radius;
	double theta1;
	double theta2;

};


}

#endif
