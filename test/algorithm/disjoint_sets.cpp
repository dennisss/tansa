#include "gtest/gtest.h"

#include <tansa/algorithm.h>

using namespace tansa;
using namespace std;


TEST(Algorithms, DisjointSets) {

	DisjointSets forest(10);

	ASSERT_NE(forest.findSet(1), forest.findSet(3));

	forest.unionSets(1, 3);

	ASSERT_EQ(forest.findSet(1), forest.findSet(3));

	ASSERT_NE(forest.findSet(1), forest.findSet(2));

}
