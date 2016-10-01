#ifndef TANSA_TRAJECTORY_H_
#define TANSA_TRAJECTORY_H_

#include <Eigen/Dense>

using namespace Eigen;

// TODO: Should eventually be Vector4d to incorporate yaw
typedef Vector3d Point;


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
 * A path that the vehicle should follow
 * They are parametrized w.r.t. time
 * Should be a three time differentiable function
 */
class Trajectory {
public:
	virtual TrajectoryState evaluate(double t) = 0;
};

/**
 * Smoothly goes in a straight line through two points
 */
class LinearTrajectory : public Trajectory {
public:

	static LinearTrajectory &compute(Point p1, double t1, Point p2, double t2);

	virtual TrajectoryState evaluate(double t);

private:
	LinearTrajectory();

	// Store coefficients for x, y, z
	VectorXd coeffs[3];
};


class CircleTrajectory : public Trajectory {
public:
	virtual TrajectoryState evaluate(double t);



};


#endif
