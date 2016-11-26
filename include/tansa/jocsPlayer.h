#ifndef TANSA_JOCSPLAYER_H
#define TANSA_JOCSPLAYER_H

#include <tansa/core.h>
#include <tansa/control.h>


#include "tansa/jocsParser.h"
#include "tansa/config.h"

namespace tansa {
	class JocsPlayer {
	public:
		void initVehicles(const std::vector<Vehicle *> &vehicles);

		// Get ready to fly (arm and takeoff to home point)
		void prepare();

		// Start flying
		void play();

		void pause();

		void land();

		// Lands and disarms all drones (Must already be paused)
		void stop();

		void step();

		void loadJocs(string jocsPath, float scale, const std::vector<unsigned> &jocsActiveIds, int start = -1);



		// TODO: Instead we should use isRunning which checks if any states are not StateInit
		bool canLoad();
		inline bool isPlaying() { return states.size() > 0 && this->states[0] == StateFlying; }
		inline bool isReady() { return states.size() > 0 && this->states[0] == StateHolding; }
		inline bool isPaused() { return this->paused; }
		inline bool isLanded() { return this->landed; }

		/**
		 * Gets the time relative to the start of the current file
		 */
		double currentTime();

		std::vector<Point> getHomes();
		std::vector<std::vector<Action*>> getActions();
		std::vector<Breakpoint> getBreakpoints();
		void cleanup();
	private:
		std::vector<Vehicle *> vehicles;
		std::vector<vehicle_config> vehicleConfigs;
		std::vector<unsigned> jocsActiveIds;

		std::vector<Breakpoint> breakpoints;
		Jocs* currentJocs = nullptr;
		std::vector<std::vector<Action*>> actions;
		std::vector<std::vector<LightAction*>> lightActions;
		std::vector<Point> homes;
		std::vector<HoverController *> hovers;
		std::vector<PositionController *> posctls;
		std::vector<LightController *> lightctls;
		std::vector<int> lightCounters;
		std::vector<Point> holdpoints;
		std::vector<PlayerVehicleState> states;
		std::vector<int> plans;

		// Separate trajectories and timings for doing takeoff and landings
		std::vector<Trajectory *> transitions;
		vector<Time> transitionStarts;
		bool pauseRequested = false;
		bool paused = false;
		bool stopRequested = false;
		bool landed = false;
		Time start = Time(0,0); // TODO: I will also need a time offset
		Time pauseOffset = Time(0,0);
		double timeOffset = 0.0;
		std::vector<int> pauseIndices;
		int stepTick = 0;
		std::vector<int> startIndices;
		std::vector<int> endIndices;
		double startOffset = 0.0;

		double getNextBreakpointTime(double lastTime);
		double getBreakpointTime(unsigned breakpointNumber);
		double getBreakpointTime(std::string breakpointName);
		unsigned getBreakpointNumber(double startTime);
		Point getDroneLocationAtTime(double startTime, unsigned droneId);
		bool isMotionAction(Action* a);
		void createBreakpointSection(int startPoint, int endPoint = -1);
	};
}
#endif //TANSA_JOCSPLAYER_H
