#include "gtest/gtest.h"

#include <tansa/jocsParser.h>
#include <tansa/jocsPlayer.h>

using namespace tansa;

TEST(Jocs, Parse) {

	Jocs *j = Jocs::Parse("data/singleDrone.jocs", 1.0);

	std::vector<Point> pts = j->GetHomes();

	ASSERT_EQ(pts.size(), 1);
	ASSERT_TRUE(pts[0].isApprox(Vector3d(0, 0, 2), 0.001));

}
