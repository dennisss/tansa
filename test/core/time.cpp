#include "gtest/gtest.h"

#include <tansa/time.h>
#include <unistd.h>

// Just making sure that we can correctly measure time differences
TEST(Time, UnitDurations) {

	int udur = 1000000 / 4;

	Time start = Time::now();
	usleep(udur);
	Time diff = Time::now().since(start);

	ASSERT_LT(abs(0.25 - diff.seconds()), 0.025);
	ASSERT_LT(abs(udur - diff.micros()), 25000);
}
