#ifndef TANSA_TIME_H_
#define TANSA_TIME_H_

#include <stdint.h>
#include <time.h>

namespace tansa {

/**
 * An implementation of a common way to represent and measure
 */
class Time {

public:
	//Time();
	Time() : Time(0, 0) { }
	Time(int secs, int nsecs);
	Time(double secs);

	/**
	 * Gets the current time object
	 */
	static Time now();


	static Time realNow();

	/**
	 * Get how much time has elapsed since the other point
	 */
	Time since(const Time &other) const;

	/**
	 * Get time elapsed since the start of the program
	 */
	Time sinceStart();


	friend Time operator+(Time lhs, const Time &rhs);


	/**
	 * Gets the time value in nanoseconds
	 */
	uint64_t nanos() const;

	/**
	 * Gets the time value in microseconds
	 */
	uint64_t micros() const;

	/**
	 * Gets the time value in milliseconds
	 */
	uint64_t millis() const;

	/**
	 * Gets the time in seconds (with millisecond floating point precision)
	 * Note: only use this on a time difference. This may not be accurate from a clock time
	 */
	double seconds() const;


	/**
	 * Sets the current time to be some other time.
	 * Optionally 'factor' specifies the time scale compared to real time. i.e. 0.5 means that 2 seconds go by for every 1 second of real time.
	 */
	static void setTime(const Time &t, double factor = 0);

private:

	struct timespec val;
};

Time operator+(Time lhs, const Time &rhs);


/**
 * For operating a loop at a certain frequency
 */
class Rate {

public:
	/**
	 * Initializes a new rate tracker
	 * Note: you should initialize it immediately before starting the first iteration of the loop
	 */
	Rate(unsigned int hz);

	/**
	 * Call this at the end of each loop cycle to proceed to the next
	 */
	void sleep();

private:
	unsigned int hz;
	unsigned int uperiod;
	Time lasttime;

};


// TODO: Add in Timer class


}

#endif
