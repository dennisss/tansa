#include <tansa/time.h>

#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifdef WIN32
#include <Windows.h>
#endif

#include <iostream>

using namespace std;



namespace tansa {


#ifdef WIN32
static LARGE_INTEGER clock_frequency; // On windows, this is the number of clock ticks per second
#endif
// Real time at which the
static Time starttime(0,0);

// Whether or not we are using a simulation timer
static bool simTimeValid;
static Time simTime(0,0);
static Time simRefTime(0,0); // The wall time
static float simFactor;


/*
	Gets the current time
	- if since_epoch is set to true, then the current calendar time/time since epoch will be used. this time is less precise
	- otherwise, the best monotonic clock source will be used

	Credit to https://gist.github.com/jbenet/1087739 for some of this function
*/
void current_time(struct timespec *ts, bool since_epoch = false) {

#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), since_epoch? CALENDAR_CLOCK : SYSTEM_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	ts->tv_sec = mts.tv_sec;
	ts->tv_nsec = mts.tv_nsec;
#elif WIN32
	LARGE_INTEGER ticks;
	QueryPerformanceCounter(&ticks);

	uint64_t nsecs = ticks.QuadPart / (clock_frequency.QuadPart * 1000000000);
	ts->tv_sec = nsecs / 1000000000;
	ts->tv_nsec = nsecs % 1000000000;
#else
	clock_gettime(since_epoch? CLOCK_REALTIME : CLOCK_MONOTONIC, ts);
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

void time_init() {

#ifdef WIN32
	QueryPerformanceFrequency(&clock_frequency);
#endif

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
	double intpart;
	double frac = modf(seconds, &intpart);

	this->val.tv_sec = intpart;
	this->val.tv_nsec = frac * 1000000000;
}


Time Time::now() {
	Time t = Time::realNow();

	if(simTimeValid) {
		Time dt = t.since(simRefTime);
		dt = dt.scale(simFactor);

		Time stime;
		timespec_add(&stime.val, &dt.val, &simTime.val);
		return stime;
	}

	return t;
}

Time Time::realNow() {
	struct timespec t;
	current_time(&t, true);

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

Time Time::add(const Time &rhs) {
	Time sum;
	timespec_add(&sum.val, &this->val, &rhs.val);
	return sum;
}

Time Time::subtract(const Time &rhs) {
	Time diff;
	timespec_subtract(&diff.val, &rhs.val, &this->val);
	return diff;
}

uint64_t Time::nanos() const {
	return (uint64_t) val.tv_nsec + ((uint64_t) val.tv_sec * 1000000000);
}

uint64_t Time::micros() const {
	return ((uint64_t) val.tv_nsec / 1000) + ((uint64_t) val.tv_sec * 1000000);
}

uint64_t Time::millis() const {
	return ((uint64_t) val.tv_nsec / 1000000) + ((uint64_t) val.tv_sec * 1000);
}

double Time::seconds() const {
	return ((double) val.tv_sec) + (((double) val.tv_nsec) / 1000000000.0);
}

std::string Time::dateString() const {
	struct tm t;

	char buf[64];

	tzset();
	if(localtime_r(&(val.tv_sec), &t) == NULL)
		return "Unknown-" + std::to_string(val.tv_sec);

	strftime(buf, sizeof(buf), "%Y%m%d-%H_%M_%S", &t);

	return std::string(buf);
}


Rate::Rate(unsigned int hz) : lasttime(0,0) {
	this->hz = hz;
	this->lasttime = Time::now();
	this->uperiod = 1000000 / hz;
}

#define MIN(a,b) (((a)<(b))?(a):(b))

void Rate::sleep() {

	uint64_t done = Time::now().since(lasttime).micros();

	// Number of cycles completed since last time
	unsigned cycles = done / uperiod;

	// Sleep if we are still undertime
	if(cycles == 0) {
		usleep(uperiod - done);
		cycles++;
	}
	else {
		//cerr << "Loop too slow" << endl;
	}

	Time dt = Time(0, 1000*uperiod).scale(cycles);
	lasttime = lasttime.add( dt );
}

}
