#ifndef TANSA_TIME_H_
#define TANSA_TIME_H_

#include <stdint.h>

/**
 * An implementation of a common way to represent and measure 
 */
class Time {

public:

	/**
	 * Gets the current time object
	 */
	static Time now();

	/**
	 * Gets the time value in microseconds since program startup
	 */
	uint64_t micros();


	/**
	 * Warning: don't call this directly. It is called by tansa::init()
	 */
	void init();

private:

	uint64_t microtime;



};


#endif
