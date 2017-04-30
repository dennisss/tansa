#include <tansa/manager.h>


namespace tansa {

/*
	Move the whole takeoff and land stuff here
*/


void JocsPlayer::initVehicles(const std::vector<Vehicle::Ptr> &vehicles) {
	this->vehicles = vehicles;

	int n = vehicles.size();

	hovers.resize(n);
	for(int i = 0; i < n; i++) {
		hovers[i] = new HoverController(vehicles[i]);
	}

	posctls.resize(n);
	for(int i = 0; i < n; i++) {
		posctls[i] = new PositionController(vehicles[i]);
	}

	lightCounters.resize(n);
	for(auto &light : lightCounters) {
		light.resize(LightController::NUM_LIGHTS, 0);
	}

	lightctls.resize(n);
	for(int i = 0; i < n; i++) {
		lightctls[i] = new LightController(vehicles[i]);
	}

	states.resize(n);
	for(auto& state : states){
		state = StateInit;
	}

	transitionStarts.resize(n, Time(0,0));
}

void Manager::prepare() {

	// TODO: Enforce that a program is active

	for(const auto& v : vehicles){
		if(!v->tracking) {
			printf("Don't have mocap tracking for at least one drone\n");
			return;
		}
	}

	for(const auto &v : vehicles) {
		// TODO: Consider safety checks here

	}

	for(auto s : states) {
		if(s != StateInit) {
			printf("Cannot prepare: Some drones not in initial state\n");
			return;
		}
	}

	bool inPlace = true;
	int n = jocsActiveIds.size();
	for(int i = 0; i < n; i++) {
		Vector3d start = vehicles[i]->state.position,
				 end = homes[jocsActiveIds[i]];
		end.z() = 0;

		double e = (start - end).norm();
		if(e > 0.5) {
			printf("Role %d not in place. Off by %.2fm\n", i, e);
			inPlace = false;
		}
	}

	if(!inPlace) {
		return;
	}


	for(auto &s : states) {
		s = StateArming;
	}
}


void Manager::play() {

	// TODO: We should also check that the drones are in the right place (a Stop may get them to the wrong position)

	for(auto s : states) {
		if(s != StateHolding) {
			printf("Cannot play: Some drones not ready\n");
			return;
		}
	}

	if (paused) {
		paused = false;
		pauseRequested = false;
		stopRequested = false;
		timeOffset += Time::now().since(pauseOffset).seconds();
		int n = jocsActiveIds.size();
		for (int i = 0; i < n; i++) {
			plans[i] = pauseIndices[i];
		}
	} else {
		start = Time::now();
		reset();
	}

	for(auto &s : states) {
		s = StateFlying;
	}
}

double Manager::currentTime() {
	if(!this->isPlaying()) {
		return -1;
	}

	return Time::now().since(start).seconds() + startOffset;
}

void Manager::failsafe() {
	for(auto &s : states) {
		s = StateFailsafe;
	}
}

void Manager::land() {
	for(auto s : states) {
		if(s != StateHolding) {
			printf("Cannot land: Some drones not holding position\n");
			return;
		}
	}

	for(int i = 0; i < states.size(); i++) {
		states[i] = StateLanding;

		Point lastPoint = holdpoints[jocsActiveIds[i]];
		Point groundPoint = lastPoint; groundPoint.z() = 0;
		double dur = lastPoint.z() / VEHICLE_DESCENT_MS;
		transitions[i] = Trajectory::Ptr( new LinearTrajectory(lastPoint, 0, groundPoint, dur) );
		transitionStarts[i] = Time::now();
	}
}


void Manager::rearrange() {
	for(auto s : states) {
		if(s != StateInit) {
			printf("Cannot prepare: Some drones not in initial state\n");
			return;
		}
	}


	// Distance between each vehicle and each home
	MatrixXd D(vehicles.size(), homes.size());
	for(int i = 0; i < vehicles.size(); i++) {
		Vector3d vp = vehicles[i]->state.position;
		vp.z() = 0;

		for(int j = 0; j < homes.size(); j++) {
			Vector3d hp = homes[j];
			hp.z() = 0;

			D(i, j) = (vp - hp).norm();
		}
	}

	vector<int> c;


	AssignmentSolver s;
	s.solve(D, &c);

	for(int i = 0; i < c.size(); i++) {
		jocsActiveIds[i] = c[i];
	}
}


void Manager::stop() {

	// TODO: Have this be a hard stop

	if (!paused) {
		printf("Must already be paused.");
		return;
	}
	stopRequested = true;
}


// TODO: Somewhere we should constantly verify that the drones are still armed and in the right mode as
void Manager::step() {

	int n = jocsActiveIds.size();

	// Check for state transitions
	if (states[0] == StateInit) {
		// No implicit transitions
	} else if (states[0] == StateArming) {
		// Once all are armed, take them all off to the current position

		bool allGood = true;
		for(int i = 0; i < n; i++) {
			if (vehicles[i]->mode != "offboard" || !vehicles[i]->armed) {
				allGood = false;
				break;
			}
		}

		if (allGood) {
			start = Time::now();
			for(auto& state : states) {
				state = StateReady; // TODO: Shouldn't this be StateReady?
			}

			transitions.resize(n);
			for(int i = 0; i < n; i++) {
				Vector3d startPosition = vehicles[i]->state.position,
				endPosition = homes[jocsActiveIds[i]];

				double dur = (startPosition - endPosition).norm() / VEHICLE_ASCENT_MS;
				states[i] = StateTakeoff;
				transitionStarts[i] = start;
				transitions[i] = Trajectory::Ptr( new LinearTrajectory(startPosition, 0, endPosition, dur) );
			}
			// TODO: Only grab the ones for the active drones
			holdpoints = homes;

			// Opening file for logging data
			logfile.open("log/" + Time::realNow().dateString() + ".csv", ofstream::out);

			return;
		}
	}

	// Do the control loops
	for(int i = 0; i < n; i++) {
		Vehicle &v = *vehicles[i];
		auto &s = states[i];

		int chorI = jocsActiveIds[i];

		// Failsafe, land on motion capture lose
		if(s != StateInit) {
			if(!v.connected) {
				// Flip out to the user!
				cout << "UNCONTROLLABLE: VEHICLE NOT CONNECTED: THIS IS BAD!!" << endl;
			}
			else if(!v.tracking && s != StateFailsafe) {
				cout << "Not tracking, entering failsafe mode" << endl;
				s = StateFailsafe;
			}
			else if(v.overactuated) {
				s = StateFailsafe;
			}
		}

		if (s == StateArming) {
			// Lower frequency state management
			if(stepTick % 50 == 0) {
				if (v.mode != "offboard") {
					v.set_mode("offboard");
					printf("Setting mode\n");
				} else if (!v.armed) {
					v.arm(true);
					printf("Arming mode\n");
				}
			}


			// Do nothing
			v.setpoint_zero();
		} else if (s == StateTakeoff) {
			double t = Time::now().since(transitionStarts[i]).seconds();

			// If the drones started on the ground and overshot the target, switch to hover
			bool overshot = v.state.position.z() > holdpoints[chorI].z();

			if(transitions[i] == NULL || t >= transitions[i]->endTime() || overshot) {
				s = StateHolding;
			}

			posctls[i]->track(transitions[i]);
			posctls[i]->control(t);
		} else if (s == StateReady) {
			// Do nothing
			v.setpoint_zero();
		} else if (s == StateHolding) {
			double t = Time::now().seconds();

			if (pauseRequested) {
				cout << "Transitioning to paused, t = " << Time::now().since(start).seconds() - timeOffset << endl;
				paused = true;
				pauseRequested = false;
				pauseOffset = Time::now();
			}

			if (paused && (stopRequested || Time::now().since(pauseOffset).seconds() > 20.0)) {
				string message = stopRequested ? "Stop requested" : "Paused for more than 20 seconds";
				cout << message + ", attempting to land." << endl;
				paused = false;
				stopRequested = false;
				this->land();
				return;
			}
			// Do a hover
			hovers[i]->setPoint(holdpoints[chorI]); // TODO: Only do this on transitions (when holdpoints changes)
			hovers[i]->control(t);
		} else if (s == StateFlying) {
			this->log();


			// This was all moved to individual programs

		} else if (s == StateLanding) {
			std::vector<int> light_states;
			light_states.resize(LightController::MAX_LIGHTS, 0);
			v.set_lighting(light_states);
			double t = Time::now().since(transitionStarts[i]).seconds();

			// Special behavior near the ground. Stop trying to descend if we are hitting ground effects
			bool groundEffect = (v.state.position.z() < 0.20 && (v.state.velocity.norm() < 0.08)) || v.state.position.z() < 0.15;

			if(t < transitions[i]->endTime() && !groundEffect) {
				posctls[i]->track(transitions[i]);
				posctls[i]->control(t);
			}
			// Disarm
			else if(v.armed){
				v.setpoint_zero();
				v.arm(false);
			}
			// Reset state machine
			else {
				landed = true;
				s = StateInit;
			}

		}
		else if(s == StateFailsafe) {

			// TODO: If still tracking, try a nice controlled landing
			// TODO: Geofence based on covariance to auto-hardkill
			while(v.mode == "offboard") {
				v.set_mode("landing");
			}
		}
	}

	stepTick++;

}


void Manager::cleanup() {
	for (int i = 0; i < posctls.size(); i++) {
		delete posctls[i];
	}

	for (int i = 0; i < lightctls.size(); i++) {
		delete lightctls[i];
	}

	for (int i = 0; i < hovers.size(); i++) {
		delete hovers[i];
	}
}


// TODO: Just integrate this into the status message system used by the gui
// We want to have one standardized data implementation
void Manager::log() {

	// TODO: Log based on program time
	double t = Time::now().since(start).seconds();

	logfile << t << ",";
	for(int i = 0; i < vehicles.size(); i++) {
		Vehicle *v = vehicles[i];

		// TODO: this needs to sometimes use the hover controller
		// Also
		Vector3d target = posctls[i]->getTargetState(t).position;
		logfile << target.x() << "," << target.y() << "," << target.z() << ",";


		Vector3d pos = v->state.position;
		logfile << pos.x() << "," << pos.y() << "," << pos.z() << ",";
	}

	logfile << endl;
	logfile.flush();

}

}
