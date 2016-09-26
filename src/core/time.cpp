#include <tansa/time.h>

#include <time.h>


// Precise time measured from arbitrary time in the past representing the
static struct timespec time0;

// Whether or
static bool faketime;


void Time::init() {


	clock_gettime(CLOCK_MONOTONIC, &time0);


}
