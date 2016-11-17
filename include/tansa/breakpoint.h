//
// Created by tess on 11/17/16.
//

#ifndef TANSA_BREAKPOINT_H
#define TANSA_BREAKPOINT_H

#include <memory>

namespace tansa {

typedef unsigned breakpointId;

class Breakpoint {

public:
	Breakpoint(std::string na, unsigned nu, double st) : name(na), number(nu), startTime(st) {}
	virtual ~Breakpoint() {}

	inline std::string GetName() { return name; }
	inline unsigned GetNumber() { return number; }
	inline double GetStartTime() { return startTime; }

private:
	std::string name;
	unsigned number;
	double startTime;
};
}

#endif //TANSA_BREAKPOINT_H
