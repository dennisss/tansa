#include <tansa/mocap.h>
#include <tansa/gazebo.h>
#include <zconf.h>
#include <tansa/control.h>
#include "tansa/jocsPlayer.h"
#include <unistd.h>
#include <sys/signal.h>

namespace tansa {

	/**
	 * Begin to play the choreography
	 */
	void JocsPlayer::play() {

	}

	/**
	 * Pause the choreography
	 */
	void JocsPlayer::pause() {
		JocsPlayer::pauseRequested = true;
	}

	/**
	 * Rewind the chreography by a number of 'steps'
	 * @param steps How far back in the choreography to rewind
	 */
	void JocsPlayer::rewind(int steps) {

	}

	/**
	 * Reset the choreography back to the initial position (ie: plans[0])
	 */
	void JocsPlayer::reset() {
		JocsPlayer::resetMode = true;
	}

    /**
     * Load JOCS data from a specified path
     */
    void JocsPlayer::loadJocs(string jocsPath) {
		// TODO: Clear all these if they already have data.
		JocsPlayer::jocsData = Jocs::Parse(jocsPath);
		JocsPlayer::homes = jocsData.GetHomes();
		JocsPlayer::actions = jocsData.GetActions();
		JocsPlayer::breakpoints = jocsData.GetBreakpoints();
    }

	/**
     * Helper methods
     * TODO: write test cases (especially since the code doesn't use them yet)
     */

	double JocsPlayer::getNextBreakpointTime(double lastTime) {
		unsigned breakpointsLength = breakpoints.size();

		// Cycles all breakpoints
		// Assumes list of breakpoints is in time-ascending order
		for (int i = 0; i < breakpointsLength - 1; i++) {
			double ret = breakpoints[i].GetStartTime();
			if (ret >= lastTime) {
				// Return the first breakpoint whose starttime hasn't passed yet
				// or is currently happening
				return ret;
			}
		}

		return -1;
	}
	double JocsPlayer::getBreakpointTime(unsigned breakpointNumber) {
		unsigned breakpointsLength = breakpoints.size();

		// Cycles all breakpoints
		for (int i = 1; i < breakpointsLength - 1; i++) {
			unsigned ret = breakpoints[i].GetNumber();
			if (ret == breakpointNumber) {
				return breakpoints[i].GetStartTime();
			}
		}

		return -1;
	}
	double JocsPlayer::getBreakpointTime(std::string breakpointName) {
		unsigned breakpointsLength = breakpoints.size();

		// Cycles all breakpoints
		for (int i = 1; i < breakpointsLength - 1; i++) {
			std::string name = breakpoints[i].GetName();
			if (name.compare(breakpointName) == 0) {
				return breakpoints[i].GetStartTime();
			}
		}

		return -1;
	}
	unsigned JocsPlayer::getBreakpointNumber(double startTime) {
		unsigned breakpointsLength = breakpoints.size();

		// Cycles all breakpoints
		for (int i = 1; i < breakpointsLength - 1; i++) {
			double ret = breakpoints[i].GetStartTime();
			if (ret > startTime) {
				// Return the breakpoint that the given time is a part of
				// Also works for given times that are within a breakpoint
				// not just the start time of a breakpoint
				return breakpoints[i-1].GetNumber();
			}
		}

		return -1;
	}
	Point JocsPlayer::getDroneLocationAtTime(double startTime, unsigned droneId) {
		// This one only works if the startTime is a startTime of an action
		// Returns negative value if startTime is either between actions or out of scope

		unsigned breakpointsLength = breakpoints.size();
		unsigned actionsLength = actions[droneId].size();

		// Assume droneId is valid and use it to choose which list of actions we need to parse
		for (int j = 0; j < actionsLength; j++) {
			double actionStartTime = actions[droneId][j]->GetStartTime();
			bool isMotionAction = true; // TODO: actually check if it's a motion. For now, they're all motions.
			if (actionStartTime == startTime && isMotionAction) {
				return ((MotionAction*)actions[droneId][j])->GetStartPoint(); // TODO: check if this cast works...
			}
		}
	}
}
