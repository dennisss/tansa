#ifndef TANSA_JOCSPLAYER_H
#define TANSA_JOCSPLAYER_H

#include "tansa/jocsParser.h"

namespace tansa {
	class JocsPlayer {
	public:
		JocsPlayer(bool u) : useMocap(u) {}
		void play();
		void pause();
		void rewind(int steps);
		void reset();
		void loadJocs(string jocsPath);
		void loadConfig(string configPath);
	private:
		//Jocs jocsData;
		std::vector<Point> homes;
		std::vector<std::vector<Action*>> actions;
		std::vector<Breakpoint> breakpoints;
		bool useMocap;
		bool running = false;
		bool pauseRequested = false;
		bool resetMode = false;

		double getNextBreakpointTime(double lastTime);
		double getBreakpointTime(unsigned breakpointNumber);
		double getBreakpointTime(std::string breakpointName);
		unsigned getBreakpointNumber(double startTime);
		Point getDroneLocationAtTime(double startTime, unsigned droneId);
	};
}
#endif //TANSA_JOCSPLAYER_H
