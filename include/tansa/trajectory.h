#ifndef TANSA_TRAJECTORY_H_
#define TANSA_TRAJECTORY_H_


// Used for generating feasible trajectories
#define MAX_ACCELERATION 3

/*
	Should be a three time differentiable function

	The controller will take x(t) v(t) a(t) -> a(t)
*/
class Trajectory {
public:
	virtual Vector4d pos(double t) = 0;
	virtual Vector4d vel(double t) = 0;
	virtual Vector4d accel(double t) = 0;
};

/*

*/
class LinearTrajectory : public Trajectory {
public:

	static LinearTrajectory &compute(Vector4d p1, double t1, Vector4d p2, double t2);



};


#endif
