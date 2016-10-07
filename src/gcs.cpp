#include <tansa/core.h>
#include <tansa/time.h>
#include <tansa/vehicle.h>
#include <tansa/control.h>
#include <tansa/trajectory.h>
#include <tansa/mocap.h>

#include <signal.h>
#include <unistd.h>

#include <vector>


bool running;

void signal_sigint(int s) {
	running = false;
}

#define STATE_TAKEOFF 0
#define STATE_FLYING 1
#define STATE_LANDING 2


int multidrone_main() {

	tansa::init();


	vector<Vector3d> homes = {
		{0, -5, 1},
		{0, -3, 1},
		{0, -1, 1},
		{0, 1, 1},
		{0, 3, 1},
		{0, 5, 1}
	};


	tansa::sim_connect();

	vector<Vehicle *> vehicles(6);
	for(int i = 0; i < 6; i++) {
		vehicles[i] = new Vehicle();
		vehicles[i]->connect(14550 + i*10, 14555 + i*10);
		tansa::sim_track(vehicles[i], i);
	}


	vector<HoverController *> hovers(6);
	for(int i = 0; i < 6; i++) {
		hovers[i] = new HoverController(vehicles[i], homes[i] + Vector3d(0, 0, 1));
	}


	vector<PositionController *> posctls(6);
	for(int i = 0; i < 6; i++) {
		posctls[i] = new PositionController(vehicles[i]);
	}



	// Generate trajectories
	vector<vector<Trajectory *> > paths(6);
	for(int i = 0; i < 6; i++) {
		Vector3d mag(3, 0, 0);
		if(i % 2 == 0)
			mag *= -1;

		paths[i].push_back(new LinearTrajectory(homes[i], 0, homes[i] + mag, 5));
		paths[i].push_back(new LinearTrajectory(homes[i] + mag, 5, homes[i] - mag, 10));
		paths[i].push_back(new LinearTrajectory(homes[i] - mag, 10, homes[i] + mag, 15));
		paths[i].push_back(new LinearTrajectory(homes[i] + mag, 15, homes[i] - mag, 20));
		paths[i].push_back(new LinearTrajectory(homes[i] - mag, 20, homes[i] + mag, 25));
		paths[i].push_back(new LinearTrajectory(homes[i] + mag, 25, homes[i] - mag, 30));

	}




	int state = STATE_TAKEOFF;
	running = true;
	signal(SIGINT, signal_sigint);

	int i = 0;
	Time start(0,0);

	int planI = 0;

	Rate r(100);
	while(running) {
		// Check for state transitions
		if(state == STATE_TAKEOFF) {
			bool allGood = true;
			for(int vi = 0; vi < 6; vi++) {
				if(hovers[vi]->distance() > 0.1) {
					allGood = false;
					break;
				}
			}

			if(allGood) {
				start = Time::now();
				state = STATE_FLYING;
			}
		}


		// Do the control loops
		for(int vi = 0; vi < 6; vi++) {
			Vehicle &v = *vehicles[vi];

			if(state == STATE_TAKEOFF) {
				// Lower frequency state management
				if(i % 50 == 0) {
					if(v.mode != "offboard") {
						v.set_mode("offboard");
						printf("Setting mode\n");
					}
					else if(!v.armed) {
						v.arm(true);
						printf("Arming mode\n");
					}
				}


				hovers[vi]->control(0.0);
			}
			else if(state == STATE_FLYING) {

				double t = Time::now().since(start).seconds();
				if(t >= paths[vi][planI]->endTime())
					planI++;

				if(planI >= paths[vi].size()) {
					state = STATE_LANDING;
					v.land();
					continue;
				}

				Trajectory *cur = paths[vi][planI];
				posctls[vi]->track(cur);
				posctls[vi]->control(t);
			}


		}

		r.sleep();
		i++;
	}






	return 0;

}


int main(int argc, char *argv[]) {

	return multidrone_main();

	bool mocap_enabled = false,
		 sim_enabled = true;

	// TODO: Parse arguments here
	// -mocap to start mocap
	// -sim to use gazebo listeners

	tansa::init();


	Mocap *mocap = NULL;

	Vehicle v;
	v.connect();


	// TODO: Ensure only one of these is enabled at a time
	if(sim_enabled) {
		tansa::sim_connect();
		tansa::sim_track(&v, 0);
	}
	if(mocap_enabled) {
		string client_addr = "192.168.2.1";
		mocap = new Mocap();
		mocap->connect(client_addr);
		mocap->track(&v, 1);
	}

	running = true;
	signal(SIGINT, signal_sigint);

	// Points of a square
	vector<Point> points = {
		{2, 2, 1},
		{2, -2, 1},
		{-2, -2, 1},
		{-2, 2, 1},
		{0, 0, 1}
	};

	// Point in the air where the clock starts
	Point home(0, 0, 1);


	int state = STATE_TAKEOFF;

	int i = 0;

/*
	// For sample lighting demo
	float level = 0;
	float dl = 0.005;
*/

	HoverController hover(&v, home);
	PositionController posctl(&v);


	vector<Trajectory *> plan;
	int planI = 0; // Current part of the plan to execute
	double curT = 0.0; // Last time added to the plan (just used in the planning phase)


	CircleTrajectory circle(Point(0,0,1), 2, 0, 5.0, 2.0*M_PI, 20.0);
	TrajectoryState cS = circle.evaluate(circle.startTime());
	TrajectoryState cE = circle.evaluate(circle.endTime());

	// Enter into the circle
	plan.push_back(PolynomialTrajectory::compute(
		{ home }, 0.0,
		{ cS.position, cS.velocity, cS.acceleration }, 5.0
	));
	curT += 5;

	// Then do the full circle
	plan.push_back(&circle);
	curT = circle.endTime();

	// Exit the circle
	plan.push_back(PolynomialTrajectory::compute(
		{cE.position, cE.velocity, cE.acceleration}, curT,
		{points[0]}, curT + 2.5
	));
	curT += 2.5;

	// Finally do a square (and go back to the origin)
	for(int i = 1; i < points.size(); i++) {
		plan.push_back(new LinearTrajectory(points[i-1], curT, points[i], 5.0 + curT));
		curT += 5;
	}


	Time start(0,0);
	Rate r(100);

	while(running) {
/*
		Sample Lighting stuff

		v.set_lighting(level, level);

		level += dl;
		if(level >= 1.0 || level <= 0.0)
			dl = -dl;
*/


		if(state == STATE_TAKEOFF) {

			// Lower frequency state management
			if(i % 50 == 0) {
				if(v.mode != "offboard") {
					v.set_mode("offboard");
					printf("Setting mode\n");
				}
				else if(!v.armed) {
					v.arm(true);
					printf("Arming mode\n");
				}
			}

			hover.control(0.0);

			if(hover.distance() < 0.1) {
				start = Time::now();
				state = STATE_FLYING;
			}

		}
		else if(state == STATE_FLYING) {

			double t = Time::now().since(start).seconds();
			if(t >= plan[planI]->endTime())
				planI++;

			if(planI >= plan.size()) {
				state = STATE_LANDING;
				v.land();
				continue;
			}

			Trajectory *cur = plan[planI];
			posctl.track(cur);
			posctl.control(t);
		}
		else if(state == STATE_LANDING) {
			// TODO:
		}

		i++;

		r.sleep();
	}



	// Cleanup
	if(sim_enabled) {
		tansa::sim_disconnect();
	}



	// Stop all vehicles
	v.disconnect();

	printf("Done!\n");


}
