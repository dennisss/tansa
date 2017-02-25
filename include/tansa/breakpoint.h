#ifndef TANSA_BREAKPOINT_H
#define TANSA_BREAKPOINT_H

namespace tansa {

typedef unsigned breakpointId;

/**
 * @class Represents a breakpoint in JOCS. A point in time at which all drones have zero velocity and so
 * can be safely paused, resumed or started from.
 */
class Breakpoint {

public:
	/**
	 * Constructs a breakpoint object
	 * @param na Breakpoint name for being shown in the gui and selected.
	 * @param nu Breakpoint number for indexing the breakpoints.
	 * @param st Breakpoint start time in choreography.
	 * @return
	 */
	Breakpoint(std::string na, unsigned nu, double st) : name(na), number(nu), startTime(st) {}
	virtual ~Breakpoint() {}
	/**
	 * @return Name of the breakpoint.
	 */
	inline std::string GetName() { return name; }
	/**
	 * @return Number of the breakpoint
	 */
	inline unsigned GetNumber() { return number; }
	/**
	 * @return Start time of the breakpoint.
	 */
	inline double GetStartTime() { return startTime; }

private:
	std::string name;
	unsigned number;
	double startTime;
};
}

#endif //TANSA_BREAKPOINT_H
