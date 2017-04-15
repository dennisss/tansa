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
			t = trajs[i]->endTime();
		}
		else {
			t = t - ts; // offset the time
		}

		return trajs[i]->evaluate(t);
	}



private:
	vector<Trajectory::Ptr> trajs;

};

/**
 * A trajectory represented by evaluating a polynomial over time
 */
class PolynomialTrajectory : public Trajectory {
public:

	/**
	 * Creates a new trajectory given its coefficients
	 *
	 * @param c array of coefficients for each axis of a Point. These should be in the order 'c0 + c1*t^1 + c2*t^2' (this is the opposite of numpy). These trajectories are also internally offset such that the polynomial will always be first evaluated at zero time
	 * @param t1 start time (t = 0 in polynomial time)
	 * @param t2 end time (t = t2 - t1 in polynomial time)
	 */
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
 *
 * Inspired by the formulations in 'Dancing Quadrocopters: Trajectory Generation, Feasibility, and User Interface' by Federico Augugliaro, Angela Schoellig
 *
 */
class FourierTrajectory : public Trajectory {
public:

	/**
	 *
	 * @param A a 3xN matrix
	 * @param B a 3xN matrix
	 * @param freq
	 * @param phase
	 * @param center
	 * @param ts
	 * @param te
	 */
	FourierTrajectory(const MatrixXd &A, const MatrixXd &B, double freq, double phase, Point center, double ts, double te);
	virtual ~FourierTrajectory() {}

	virtual TrajectoryState evaluate(double t);


private:

	MatrixXd A;
	MatrixXd B;
	double freq;
	double phase;
	Point center;


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


inline Trajectory::Ptr HelixHelper(double radius,
	const Point &origin1, double theta1, double time1,
	const Point &origin2, double theta2, double time2) {

	auto line = make_shared<LinearTrajectory>(origin1, time1, origin2, time2);
	auto circle = make_shared<CircleTrajectory>(Vector3d(0,0,0), radius, theta1, time1, theta2, time2);

	return make_shared<CompoundTrajectory>(line, circle, time1, time2);
}


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

struct Color {
	/*
	* t is from 0 to 1
	*/
	Color interpolate_from_this(Color to_this, double t) {
		float t1 = (1.0f-t);
		float r_right = t1 * r;
		float g_right = t1 * g;
		float b_right = t1 * b;
		float r_left = t*to_this.r;
		float g_left = t*to_this.g;
		float b_left = t*to_this.b;
		
		return {r_right + r_left, g_right + g_left, b_right + b_left};
	}
	static Color from_8bit_colors(int ri, int gi, int bi){
		float r = ri/255.0f;
		float g = gi/255.0f;
		float b = bi/255.0f;
		bool invalid_color = false;
		if(r > 1.0){
			r = 1.0;
			invalid_color = true;
		}
		if(g > 1.0){
			g = 1.0;
			invalid_color = true;
		}
		if(b > 1.0){
			b = 1.0;
			invalid_color = true;
		}
		if(invalid_color){
			printf("Check color values, Color channel greater than 255 detected\n");
		}
		return {r,g,b};
	}
	float r;
	float g; 
	float b;
};
/**
 * Smoothly increases or decreases list intensity between two intensities
 */
class LightTrajectory {
public:
	typedef std::shared_ptr<LightTrajectory> Ptr;
	inline LightTrajectory(double si, Color sc, double st, double ei, Color ec, double et) :
			startIntensity(si), startColor(sc),startTime(st),
			endIntensity(ei), endColor(ec), endTime(et)
	{}
	virtual ~LightTrajectory() {}
	// Helper method to convert rgbi to packed int
	static int rgbiToInt(Color in, float i);
	// Gives the intensity at a given time between the start and end times
	virtual int evaluate(double t);

	inline double getStartIntensity() { return this->startIntensity; }
	inline double getStartTime() { return this->startTime; }
	inline double getEndIntensity() { return this->endIntensity; }
	inline double getEndTime() { return this->endTime; }
	inline Color getStartColor() {return this->startColor; }
	inline Color getEndColor() { return this->endColor; }
protected:
	double startIntensity, startTime, endIntensity, endTime;
	Color startColor, endColor;
};

/**
 * Smoothly increases or decreases list intensity between two intensities
 */
class StrobeTrajectory : public LightTrajectory {
public:

	inline StrobeTrajectory(double si, Color sc, double st, double ei, Color ec, double et, double bpss, double bpse) :
			LightTrajectory(si,sc,st,ei,ec,et),startBeatsPerSecond(bpss), endBeatsPerSecond(bpse) {}
	virtual ~StrobeTrajectory() {}

	// Gives the intensity at a given time between the start and end times
	virtual int evaluate(double t);


private:

	double startBeatsPerSecond;
	double endBeatsPerSecond;
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
