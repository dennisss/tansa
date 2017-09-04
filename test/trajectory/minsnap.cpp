

TEST(MinSnap, Mellinger11) {

	ConstrainedPoint c1;
	c1.constrain(PointPosition,  {0, 1, 1});

	ConstrainedPoint c2;
	c2.constrain(PointPosition,  {1, -2, 1});

	Trajectory::Ptr tr;

	cout << "A" << endl;

	/*
	bool good = compute_minsnap_optimal_mellinger11({
		ConstrainedPoint::Stationary({-3, 3, 1}),
		{ {3, 2, 1} },
		{ {3, -2, 1} },
		ConstrainedPoint::Stationary({-3, -3, 1})
	}, 0, 6, {}, &tr);
	*/


	bool good = compute_minsnap_mellinger11({
	 	{ {2*0.3048,2*0.3048,2*0.3048} },
		{ {4*0.3048,4*0.3048,4*0.3048} },
	}, {0, 5}, {}, &tr, NULL);


	cout << "B" << good << endl;

	/*
	TrajectoryState s;

	s = tr->evaluate(0);
	cout << s.position.transpose() << endl;

	s = tr->evaluate(5);
	cout << s.position.transpose() << endl;

	s = tr->evaluate(10);
	cout << s.position.transpose() << endl;

	s = tr->evaluate(15);
	cout << s.position.transpose() << endl;
	*/


	for(double t = 0.0; t < tr->endTime(); t += 0.1) {
		TrajectoryState s = tr->evaluate(t);
		cout << s.position.transpose() << endl;
	}


}
