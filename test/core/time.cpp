#include "gtest/gtest.h"

#include <tansa/time.h>
#include <unistd.h>
#include <math.h>

using namespace tansa;

// Just making sure that we can correctly measure time differences
TEST(Time, UnitDurations) {

	int udur = 1000000 / 4;

	Time start = Time::now();
	usleep(udur);
	Time diff = Time::now().since(start);

	ASSERT_LT(fabs(0.25 - diff.seconds()), 0.025);
	ASSERT_LT(llabs((int64_t)udur - (int64_t)diff.micros()), 25000);
}

TEST(Time, scale) {
	Time t(2, 400000000);

	Time t2 = t.scale(2);

	ASSERT_LT(fabs(2.0*t.seconds() - t2.seconds()), 0.001);

}


TEST(Rate, LowFrequencyDrift) {

	Time t = Time::realNow();
	Rate r(10);
	for(int i = 0; i < 10; i++) {
		r.sleep();
	}

	Time t2 = Time::realNow();

	ASSERT_LT(fabs(1.0 -  t2.since(t).seconds()), 0.01);

}

TEST(Rate, HighFrequencyDrift) {

	Time t = Time::realNow();
	Rate *r = new Rate(10000);
	for(int i = 0; i < 40000; i++) {
		r->sleep();
	}

	Time t2 = Time::realNow();

	ASSERT_LT(fabs(4.0 - t2.since(t).seconds()), 0.04);
}
