#ifndef TANSA_ROUTINE_H
#define TANSA_ROUTINE_H

#include "action.h"
#include "breakpoint.h"

#include <memory>
#include <string>
#include <vector>

namespace tansa {


/**
 * A generic object containing actions that should be executed by vehicles
 */
class Routine {
public:

	virtual ~Routine() {}

	typedef std::shared_ptr<Routine> Ptr;

	// TODO: Maybe decouple the scale?
	static Routine *Load(std::string path, double scale = 1.0);

	/**
	 * Determines if some path to a file has the extension of a valid routing type
	 */
	static bool IsFile(std::string path);


	double duration();

	std::vector<std::vector<Action*>> actions;
	std::vector<Breakpoint> breakpoints;
	std::vector<Point> homes;
	std::vector<std::vector<std::vector<LightAction*>>> lightActions;
	bool needConvertToMeters = false;
	bool needConvertToRadians = false;


};

struct RoutineError {
	int line;
	std::string text;


	bool operator <(const RoutineError &other) const {
        return (this->line < other.line);
    }
};

/**
 * Used to verify the correctness of a Routine by determining if it is physically possible
 */
class FeasibilityChecker {
public:
	FeasibilityChecker() {};


	bool check(Routine &r);
	bool check(Trajectory::Ptr traj, int line = -1);

	void reset() { errors.resize(0); }

	std::vector<RoutineError> errors;


};


}


#endif
