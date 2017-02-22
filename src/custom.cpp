#include <tansa/action.h>
#include <tansa/control.h>
#include <tansa/core.h>
#include <tansa/jocsParser.h>
#include <tansa/config.h>
#include <tansa/jocsPlayer.h>


#include <iostream>

using namespace std;
using namespace tansa;

#define PI 3.14149

namespace tansa {

Jocs *custom_jocs() {

	Jocs *j = Jocs::Parse("data/singleDrone.jocs", 1); //new Jocs(false, false, false);

	j->homes.resize(1);
	j->homes[0] = Vector3d(-2, 0, 1);


	Trajectory *tr;

	compute_minsnap_mellinger11({
		ConstrainedPoint::Stationary({-2, 0, 1}),
		{ {0, 1, 1} },
		{ {1, -2, 1} },
		ConstrainedPoint::Stationary({2, -1, 1})
	}, { 0, 2, 4, 6 }, {}, &tr, NULL);


	j->actions[0].resize(0);
	j->actions[0].push_back(new MotionAction(
		0,
		tr,
		ActionTypes::Line
	));


	//j->homes[0] = Vector3d(0, 0, 0.5);
	//j->homes[1] = Vector3d(0, -0.5, 0.5);

//	string name = "a";
//	j->breakpoints.push_back(Breakpoint(name, 1, 0.0));

	//j->actions.resize(2);
	//j->actions[0].resize(0);


	/*
	AngleAxis<double> aa(PI/2, Vector3d(0,1,0));
	auto linez = new LinearTrajectory(Vector3d(0,0,2), 5, Vector3d(6,0,2), 30);
	auto circle = new CircleTrajectory(Vector3d(0,0,0), 1, 0, 5, 8*PI, 30);
	auto rotCircle = new TransformedTrajectory(circle, aa.toRotationMatrix(), Vector3d(0,0,0), 5, 30);
	auto t2 = new CompoundTrajectory(linez, rotCircle, 5, 30);
	*/

	/*
	auto linez = new LinearTrajectory(Vector3d(0,0,1), 5, Vector3d(0,0,4), 30);
	auto circle = new CircleTrajectory(Vector3d(0,0,0), 3, 0, 5, 6*PI, 30);

	auto t2 = new CompoundTrajectory(linez, circle, 5, 30);
	*/

	/*
	auto t2 = new EllipseTrajectory(Vector3d(0,0,1), 4, 2, PI, 5, 4*PI, 30);

	*/

	/*

	for(int i = 0; i < 2; i++) {
		auto linez = new LinearTrajectory(Vector3d(0,0, 0.5), 5, Vector3d(0,0,2), 30);
		auto circle = new CircleTrajectory(Vector3d(0,0,0), 1, i*PI, 5, 6*PI + i*PI, 30);

		auto t2 = new CompoundTrajectory(linez, circle, 5, 30);

		auto t = PolynomialTrajectory::compute(
			{j->homes[i]},
			0,
			{t2->evaluate(5).position, t2->evaluate(5).velocity, t2->evaluate(5).acceleration},
			5
		);

		j->actions[i].push_back(new MotionAction(
			0,
			t,
			ActionTypes::Transition
		));

		j->actions[i].push_back(new MotionAction(
			0,
			t2,
			ActionTypes::Line
		));
	}

	*/

	return j;
}

}
