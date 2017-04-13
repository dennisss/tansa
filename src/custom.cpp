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


#define CUSTOM_ARC1

namespace tansa {

Jocs *custom_jocs() {

	Jocs *j = Jocs::Parse("data/singleDrone.jocs", 1); //new Jocs(false, false, false);

#ifdef CUSTOM_SPIRAL
	j->homes.resize(1);
	j->homes[0] = Vector3d(0, 0, 1);


	Trajectory::Ptr tr( new SpiralTrajectory(Vector3d(0, 0, 1), 3, 0, 0, 6*PI, 20) );

	j->actions[0].resize(0);
	j->actions[0].push_back(new MotionAction(
		0,
		tr,
		ActionTypes::Line
	));
#endif

#ifdef CUSTOM_ARC1
	//j->homes.resize(1);
	//j->homes[0] = Vector3d(-1.5, 0, 1);

	auto line = make_shared<LinearTrajectory>(Vector3d(0, 0, 2), 42.5, Vector3d(-1.5, 0, 1), 47.5);
	j->actions[0].push_back(new MotionAction(
		0,
		line,
		ActionTypes::Line
	));



	Trajectory::Ptr tr;


	double arcStart = 47.5;
	compute_minsnap_mellinger11({
		ConstrainedPoint::Stationary({-1.5, 0, 1}),
		{ {0, 1, 0.5} },
		{ {1, -1.5, 1.5} },
		ConstrainedPoint::Stationary({1.5, 0, 2})
	}, { arcStart + 0, arcStart + 2, arcStart + 3, arcStart + 5 }, {}, &tr, NULL);


	//j->actions[0].resize(0);
	j->actions[0].push_back(new MotionAction(
		0,
		tr,
		ActionTypes::Line
	));

	auto line2 = make_shared<LinearTrajectory>(Vector3d(1.5, 0, 2), arcStart + 5, Vector3d(0, 0, 2), arcStart + 10);
	j->actions[0].push_back(new MotionAction(
		0,
		line2,
		ActionTypes::Line
	));

	double cTime = arcStart + 10.0;



	auto circle = make_shared<CircleTrajectory>(Vector3d(0,0,2), 1, 0, cTime + 2, 4*PI, cTime + 2 + 5);

	auto cS = circle->evaluate(circle->startTime());
	auto cE = circle->evaluate(circle->endTime());

	auto cTran = PolynomialTrajectory::compute(
		{{0, 0, 2}},
		cTime,
		{cS.position, cS.velocity, cS.acceleration},
		cTime + 2
	);

	auto cTran2 = PolynomialTrajectory::compute(
		{cE.position, cE.velocity, cE.acceleration},
		cTime + 7,
		{{0, 0, 2}},
		cTime + 9
	);


	j->actions[0].push_back(new MotionAction(
		0,
		cTran,
		ActionTypes::Transition
	));

	j->actions[0].push_back(new MotionAction(
		0,
		circle,
		ActionTypes::Line
	));

	j->actions[0].push_back(new MotionAction(
		0,
		cTran2,
		ActionTypes::Transition
	));




#endif

#ifdef CUSTOM_HELIX
	j->homes.resize(2);
	j->homes[0] = Vector3d(0, 0.5, 0.5);
	j->homes[1] = Vector3d(0, -0.5, 0.5);

	j->actions.resize(2);
	j->actions[0].resize(0);
	j->lightActions.resize(2);

	for(int i = 0; i < 2; i++) {
		auto linez = make_shared<LinearTrajectory>(Vector3d(0,0, 0.5), 5, Vector3d(0,0,2), 30);
		auto circle = make_shared<CircleTrajectory>(Vector3d(0,0,0), 1, i*PI, 5, 6*PI + i*PI, 30);

		auto t2 = make_shared<CompoundTrajectory>(linez, circle, 5, 30);

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
#endif

#ifdef CUSTOM_ARC2
	j->homes.resize(1);
	j->homes[0] = Vector3d(-2, 0, 1);


	Trajectory::Ptr tr;

	compute_minsnap_mellinger11({
		ConstrainedPoint::Stationary({-2, 0, 1}),
		{ {0, 1, 2} },
		{ {-1, 1, 1} },
		ConstrainedPoint::Stationary({2, -1, 1})
	}, { 0, 4, 8, 12 }, {}, &tr, NULL);


	j->actions[0].resize(0);
	j->actions[0].push_back(new MotionAction(
		0,
		tr,
		ActionTypes::Line
	));
#endif


#ifdef CUSTOM_VERT_ELLIPSE
	j->homes.resize(1);
	j->homes[0] = Vector3d(-2, 0, 1);


	double time1 = 3;
	double time2 = 7;

	Trajectory::Ptr t2 = make_shared<EllipseTrajectory>( Vector3d(0, 0, 0), 1, 1.5, 0, time1, 2*PI, time2 );


	Matrix3d m = Quaterniond::FromTwoVectors(Vector3d(0,0,1), Vector3d(0, 1, 0)).toRotationMatrix();
	Vector3d p = Vector3d(0, 0, 2);
	t2 = Trajectory::Ptr( new TransformedTrajectory(
		t2,
		m,
		p,
		time1, time2
	) );

	auto t = PolynomialTrajectory::compute(
		{j->homes[0]},
		0,
		{t2->evaluate(time1).position, t2->evaluate(time1).velocity, t2->evaluate(time1).acceleration},
		time1
	);

	j->actions[0].resize(0);
	j->actions[0].push_back(new MotionAction(
		0,
		t,
		ActionTypes::Transition
	));

	j->actions[0].push_back(new MotionAction(
		0,
		t2,
		ActionTypes::Line
	));
#endif


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

	FeasibilityChecker fc;
	fc.check(*j);

	return j;
}

}
