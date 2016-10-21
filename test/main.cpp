#include "gtest/gtest.h"

#include <cstdlib>
#include <ctime>

int main(int argc, char **argv) {
	srand(static_cast<unsigned>(time(0)));

	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
