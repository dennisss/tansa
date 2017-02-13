#include <tansa/time.h>

#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

namespace tansa {

// Real time at which the
static Time starttime(0,0);

// Whether or not we are using a simulation timer
static bool simTimeValid;
static Time simTime(0,0);
static Time simRefTime(0,0); // The wall time
static float simFactor;



/* Credit to https://gist.github.com/jbenet/1087739 for this function */
void current_utc_time(struct timespec *ts) {

#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	ts->tv_sec = mts.tv_sec;
	ts->tv_nsec = mts.tv_nsec;
#else
	clock_gettime(CLOCK_REALTIME, ts);
#endif

}

void timespec_add(struct timespec *result, const struct timespec *a, const struct timespec *b) {
	result->tv_sec = a->tv_sec + b->tv_sec + (a->tv_nsec + b->tv_nsec >= 1000000000? 1 : 0);
	result->tv_nsec = (a->tv_nsec + b->tv_nsec) % 1000000000;
}

// computes stop - start
void timespec_subtract(struct timespec *result, const struct timespec *start, const struct timespec *stop) {
	if ((stop->tv_nsec - start->tv_nsec) < 0) {
		result->tv_sec = stop->tv_sec - start->tv_sec - 1;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
	} else {
		result->tv_sec = stop->tv_sec - start->tv_sec;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec;
	}
}

// TODO: THis shouldn't change the original timespec data
void timespec_scale(struct timespec *ts, double factor) {
	ts->tv_nsec *= factor;
	ts->tv_sec *= factor;
}

void time_init() {

	simTimeValid = false;
	// This should force the usage of the wall clock
	starttime = Time::realNow();
}

/*
Time::Time() {

}
*/

Time::Time(int secs, int nsecs) {
	this->val.tv_sec = secs;
	this->val.tv_nsec = nsecs;
}

Time::Time(double seconds) {
	this->val.tv_sec = seconds;
	this->val.tv_nsec = seconds * 1000000000;
}


Time Time::now() {
	Time t = Time::realNow();

	if(simTimeValid) {
		Time dt = t.since(simRefTime);
		timespec_scale(&dt.val, simFactor);

		Time stime;
		timespec_add(&stime.val, &dt.val, &simTime.val);
		return stime;
	}

	return t;
}

Time Time::realNow() {
	struct timespec t;
	current_utc_time(&t);

	Time ot;
	ot.val = t;
	return ot;
}

void Time::setTime(const Time &t, double factor) {
	simTimeValid = true;
	simTime = t;
	simRefTime = Time::realNow();
	simFactor = factor;
}

Time Time::since(const Time &other) const {
	Time diff;
	timespec_subtract(&diff.val, &other.val, &this->val);
	return diff;
}

Time Time::sinceStart() {
	return since(starttime);
}

uint64_t Time::nanos() {
	return (uint64_t) val.tv_nsec + ((uint64_t) val.tv_sec * 1000000000);
}

uint64_t Time::micros() const {
	return ((uint64_t) val.tv_nsec / 1000) + ((uint64_t) val.tv_sec * 1000000);
}

uint64_t Time::millis() {
	return ((uint64_t) val.tv_nsec / 1000000) + ((uint64_t) val.tv_sec * 1000);
}

double Time::seconds() {
	return ((double) val.tv_sec) + (((double) val.tv_nsec) / 1000000000.0);
}

Rate::Rate(unsigned int hz) : lasttime(0,0) {
	this->hz = hz;
	this->lasttime = Time::now();
	this->uperiod = 1000000 / hz;
}

#define MIN(a,b) (((a)<(b))?(a):(b))

void Rate::sleep() {

	uint64_t done = Time::now().since(lasttime).micros();

	usleep(MIN(uperiod, uperiod - done));

	lasttime = Time::now();
}

}
