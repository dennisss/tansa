#include <tansa/mocap.h>
#include <tansa/gazebo.h>
#include <tansa/core.h>
#include <zconf.h>
#include <tansa/control.h>
#include "tansa/jocsPlayer.h"

namespace tansa {

	std::vector<Point> JocsPlayer::getHomes() {
		return homes;
	}

	std::vector<std::vector<Action*>> JocsPlayer::getActions() {
		return actions;
	}

	JocsPlayer::JocsPlayer(std::string jocsPath, double scale) {
		this->loadJocs(jocsPath, scale);
	}

	/**
	 * Load JOCS data from a specified path
	 */
	void JocsPlayer::loadJocs(std::string jocsPath, double scale) {
		// TODO: Clear all these if they already have data.
		currentJocs = Jocs::Parse(jocsPath, scale);
		homes = currentJocs->GetHomes();
		actions = currentJocs->GetActions();
	}

	void JocsPlayer::initControllers(int n, std::vector<Vehicle *> vehicles, std::vector<unsigned> jocsActiveIds) {
		// TODO: Have a better check for mocap initialization/health
		sleep(15);

		hovers.resize(n);
		for(int i = 0; i < n; i++) {
			hovers[i] = new HoverController(vehicles[i], homes[jocsActiveIds[i]]);
		}

		posctls.resize(n);
		for(int i = 0; i < n; i++) {
			posctls[i] = new PositionController(vehicles[i]);
		}


		takeoffs.resize(n);
		for(int i = 0; i < n; i++) {
			takeoffs[i] = new LinearTrajectory(vehicles[i]->state.position, 0, homes[jocsActiveIds[i]], 10.0);
		}

		states.resize(n);
		for(auto& state : states){
			state = STATE_INIT;
		}

		plans.resize(n);
	}

	/**
	 * Play one 'step' in the choreography
	 */
	Time JocsPlayer::play(std::vector<Vehicle *> vehicles, Time start, int i, int n, int &numLanded, bool &running, std::vector<unsigned> jocsActiveIds) {
		double t = Time::now().since(start).seconds();

		// Check for state transitions
		if (states[0] == STATE_INIT) {

			bool allGood = true;
			for(int vi = 0; vi < n; vi++) {
				if (vehicles[vi]->mode != "offboard" || !vehicles[vi]->armed) {
					allGood = false;
					break;
				}
			}

			if (allGood) {
				start = Time::now();
				for(auto& state :states) {
					state = STATE_TAKEOFF;
				}
			}

		} else if (states[0] == STATE_TAKEOFF) {
			if (t >= 10.0) {
				for(auto& state : states){
					state = STATE_FLYING;
				}
				start = Time::now();
			}
		}


		// Do the control loops
		for(int vi = 0; vi < n; vi++) {
			Vehicle &v = *vehicles[vi];

			if (states[vi] == STATE_INIT) {
				// Lower frequency state management
				if (i % 50 == 0) {

					if (v.mode != "offboard") {
						v.set_mode("offboard");
						printf("Setting mode\n");
					} else if (!v.armed) {
						v.arm(true);
						printf("Arming mode\n");
					}
				}


				// Do nothing
				vehicles[vi]->setpoint_accel(Vector3d(0,0,0));
			} else if (states[vi] == STATE_TAKEOFF) {
				posctls[vi]->track(takeoffs[vi]);
				posctls[vi]->control(t);
			} else if (states[vi] == STATE_FLYING) {
				if (t >= actions[jocsActiveIds[vi]][plans[vi]]->GetEndTime()) {
					if (plans[vi] == actions[jocsActiveIds[vi]].size()-1) {
						states[vi] = STATE_LANDING;
						v.land();
						numLanded++;
						if (numLanded == n) {
							running = false;
						}
						continue;
					}
					plans[vi]++;
				}
				Trajectory *cur = static_cast<MotionAction*>(actions[jocsActiveIds[vi]][plans[vi]])->GetPath();
				posctls[vi]->track(cur);
				posctls[vi]->control(t);
			}
		}
		return start;
	}

	/**
	 * Pause the choreography
	 */
	void JocsPlayer::pause() {
		printf("Pause requested!");
		pauseRequested = true;
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
		resetMode = true;
	}

	void JocsPlayer::cleanup() {
		for(int i = 0; i < posctls.size(); i++){
			delete posctls[i];
		}

		for(int i = 0; i < hovers.size(); i++){
			delete hovers[i];
		}

		for(int i = 0; i < takeoffs.size(); i++){
			delete takeoffs[i];
		}
	}
}
