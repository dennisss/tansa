#include "gtest/gtest.h"

#include <tansa/algorithm.h>

using namespace tansa;
using namespace std;


TEST(Algorithms, AssignmentSolver) {

	AssignmentSolver as;
	vector<int> c;
	double cost;

	////
	MatrixXd A(3, 3);


	A << 2, 3, 3,
		 3, 2, 3,
		 3, 3, 2;

	cost = as.solve(A, &c);
	ASSERT_EQ(cost, 6);
	ASSERT_EQ(c[0], 0);
	ASSERT_EQ(c[1], 1);
	ASSERT_EQ(c[2], 2);

	////
	A << 1, 1, 1,
		 1, 1, 1,
		 1, 1, 1;

	cost = as.solve(A, &c);
	ASSERT_EQ(cost, 3);

	////
	A << 1, 2, 3,
		 2, 4, 6,
		 3, 6, 9;

	cost = as.solve(A, &c);
	ASSERT_EQ(cost, 10);

	////
	A = MatrixXd(4, 4);
	A << 1, 2, 3, 4,
		 2, 4, 6, 8,
		 3, 6, 9, 12,
		 4, 8, 12, 16;

	cost = as.solve(A, &c);
	ASSERT_EQ(cost, 20);


}
