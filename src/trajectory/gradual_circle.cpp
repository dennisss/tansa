#include <tansa/trajectory.h>

namespace tansa {

GradualCircleTrajectory::GradualCircleTrajectory(const Point &origin, const vector<Point> &c1, double t1, const vector<Point> &c2, double t2) : Trajectory(t1, t2) {

	double radius = (c1[0] - origin).norm();
	double theta1 = atan2(c1[0].y(), c1[0].x());
	double theta2 = atan2(c2[0].y(), c2[0].x());
	circle = new CircleTrajectory(origin, radius, theta1, 0.0, theta2, 1.0);

	// The polynomial trajectory should only be along a single axis (as it will parametrize the circle in place of time)
	poly = PolynomialTrajectory::compute({
		{0, 0, 0},
		{c1[1].norm(), 0, 0},
		{c1[2].norm(), 0, 0}
	}, t1, {
		{1, 0, 0},
		{c2[1].norm(), 0, 0},
		{c2[2].norm(), 0, 0}
	}, t2);

}

GradualCircleTrajectory::~GradualCircleTrajectory() {
	delete circle;
}


TrajectoryState GradualCircleTrajectory::evaluate(double t) {

	// evaluate l(t) => time for the

	// deriv( f(g(x)) ) = f'( g(x) ) * g'(x)

	// deriv( deriv( f(x) ) ) = deriv( f'( g(x) ) ) * g'(x)  +  f'( g(x) ) * g''(x)
	//                          f''(g(x)) * g'(x)

	// g(x) and g'(x) and g'''(x)
	// TODO: These should all be number valued functions
	TrajectoryState g = poly->evaluate(t);

	// f( g(x) ),  f'( g(x) ), and f''( g(x) )
	TrajectoryState fg = circle->evaluate(g.position.norm());


	TrajectoryState s;
	s.position = fg.position;
	s.velocity = fg.velocity * g.velocity.norm();
	s.acceleration = fg.acceleration * (g.velocity.norm() * g.velocity.norm()) + fg.velocity * g.acceleration.norm();

	return s;
}

}
