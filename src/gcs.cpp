#include <tansa/core.h>
#include <tansa/time.h>
#include <tansa/vehicle.h>
#include <tansa/control.h>
#include <tansa/trajectory.h>
#include <tansa/mocap.h>
#include <tansa/gazebo.h>

#include <signal.h>
#include <unistd.h>

#include <vector>
#include <tansa/jocsParser.h>
#include <tansa/action.h>
using namespace tansa;
bool running;

void signal_sigint(int s) {
	running = false;
}

#define STATE_INIT 0
#define STATE_TAKEOFF 1
#define STATE_FLYING 2
#define STATE_LANDING 3


int multidrone_main() {

	tansa::init();

	vector<Vector3d> homes = {
		{-1, 0, 1},
		{1, 0, 1},

	/*
		{0, -5, 1},
		{0, -3, 1},
		{0, -1, 1},
		{0, 1, 1},
		{0, 3, 1},
		{0, 5, 1}
	*/
	};

	Mocap *mocap;
	GazeboConnector *gazebo;

	if(true) {
		string client_addr = "192.168.1.161";
		string server_addr = "192.168.1.150";
		mocap = new Mocap();
		mocap->connect(client_addr, server_addr);

	}
	else {
		GazeboConnector *gazebo = new GazeboConnector();
		gazebo->connect();
		// TODO: Spawn
	}

	int n = 2;


	vector<Vehicle *> vehicles(n);
	for(int i = 0; i < n; i++) {
		vehicles[i] = new Vehicle();
		vehicles[i]->connect(14550 + i*10, 14555 + i*10);
		if(true) {
			mocap->track(vehicles[i], i+1);
		}
		else {
			gazebo->track(vehicles[i], i);
		}
	}

	sleep(4);

	vector<HoverController *> hovers(n);
	for(int i = 0; i < n; i++) {
		hovers[i] = new HoverController(vehicles[i], homes[i]);
	}


	vector<PositionController *> posctls(n);
	for(int i = 0; i < n; i++) {
		posctls[i] = new PositionController(vehicles[i]);
	}



	// Generate trajectories
	vector<vector<Trajectory *> > paths(n);
	for(int i = 0; i < n; i++) {
		Vector3d mag(0, 1.5, 0);
		if(i % 2 == 0)
			mag *= -1;

		paths[i].push_back(new LinearTrajectory(homes[i], 0, homes[i] + mag, 5));
		paths[i].push_back(new LinearTrajectory(homes[i] + mag, 5, homes[i] - mag, 10));
		paths[i].push_back(new LinearTrajectory(homes[i] - mag, 10, homes[i] + mag, 15));
		paths[i].push_back(new LinearTrajectory(homes[i] + mag, 15, homes[i] - mag, 20));
		paths[i].push_back(new LinearTrajectory(homes[i] - mag, 20, homes[i] + mag, 25));
		paths[i].push_back(new LinearTrajectory(homes[i] + mag, 25, homes[i] - mag, 30));

	}

	vector<Trajectory *> takeoffs(n);
	for(int i = 0; i < n; i++) {
		takeoffs[i] = new LinearTrajectory(vehicles[i]->position, 0, homes[i], 10.0);
	}


	int state = STATE_INIT;
	running = true;
	signal(SIGINT, signal_sigint);

	int i = 0;
	Time start(0,0);

	int planI = 0;

	printf("running...\n");

	Rate r(100);
	while(running) {

		//r.sleep();
		//continue;

		double t = Time::now().since(start).seconds();


		// Check for state transitions
		if(state == STATE_INIT) {

			bool allGood = true;
			for(int vi = 0; vi < n; vi++) {
				if(vehicles[vi]->mode != "offboard" || !vehicles[vi]->armed) {
					allGood = false;
					break;
				}
			}

			if(allGood) {
				start = Time::now();
				state = STATE_TAKEOFF;
				continue;
			}

		}
		else if(state == STATE_TAKEOFF) {
			if(t >= 10.0) {
				state = STATE_FLYING;
				start = Time::now();
				continue;
			}

			/*
			bool allGood = true;
			for(int vi = 0; vi < n; vi++) {
				if(hovers[vi]->distance() > 0.1) {
					allGood = false;
					break;
				}
			}

			if(allGood) {
				start = Time::now();
				state = STATE_FLYING;
			}
			*/
		}


		// Do the control loops
		for(int vi = 0; vi < n; vi++) {
			Vehicle &v = *vehicles[vi];


			if(state == STATE_INIT) {
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


				// Do nothing
				vehicles[vi]->setpoint_accel(Vector3d(0,0,0));
			}
			else if(state == STATE_TAKEOFF) {
				posctls[vi]->track(takeoffs[vi]);
				posctls[vi]->control(t);

			}
			else if(state == STATE_FLYING) {

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

	auto data = tansa::Jocs("singleDrone.jocs");
	auto actions = data.Parse();
//	return multidrone_main();
	bool mocap_enabled = false,
		 sim_enabled = true;

	// TODO: Parse arguments here
	// -mocap to start mocap
	// -sim to use gazebo listeners

	tansa::init();


	Mocap *mocap = NULL;
	GazeboConnector *gazebo = NULL;

	Vehicle v;
	v.connect();


	// TODO: Ensure only one of these is enabled at a time
	if(sim_enabled) {
		gazebo = new GazeboConnector();
		gazebo->connect();
		gazebo->spawn({ {0,0,0} });
		gazebo->track(&v, 0);
		sleep(10); // Waiting for simulation to sync
	}
	if(mocap_enabled) {
		string client_addr = "192.168.1.161";
		string server_addr = "192.168.1.150";

		mocap = new Mocap();
		mocap->connect(client_addr, server_addr);
		mocap->track(&v, 1);

		// TODO: Have a better check for mocap initialization/health
		sleep(4);

	}

	running = true;
	signal(SIGINT, signal_sigint);

	// Points of a square
	vector<Point> points = {
		{1, 1, 1},
		{1, -1, 1},
		{-1, -1, 1},
		{-1, 1, 1},
		{0, 0, 1}
	};

	// Point in the air where the clock starts
	Point home(0, 0, 1);


	int state = STATE_INIT;

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


	CircleTrajectory circle(Point(0,0,1), 1, 0, 5.0, 2.0*M_PI, 10.0);
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


	Trajectory *takeoff = new LinearTrajectory(v.position, 0, home, 10);

	Time start(0,0);
	Rate r(100);

	while(running) {
		//r.sleep();
		//continue;

/*
		Sample Lighting stuff

		v.set_lighting(level, level);

		level += dl;
		if(level >= 1.0 || level <= 0.0)
			dl = -dl;
*/


		if(state == STATE_INIT) {

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
				else {
					state = STATE_TAKEOFF;
					start = Time::now();
				}
			}

			posctl.track(takeoff);
			posctl.control(0);

		}
		else if(state == STATE_TAKEOFF) {

			double t = Time::now().since(start).seconds();

			posctl.track(takeoff);
			posctl.control(t);


			if(t >= 10.0) {
				start = Time::now();
				state = STATE_FLYING;
				printf("Flying...\n");
			}
		}
		else if(state == STATE_FLYING) {

			double t = Time::now().since(start).seconds();
			/*if(t >= plan[planI]->endTime())
				planI++;

			if(planI >= plan.size()) {
				state = STATE_LANDING;
				v.land();
				continue;
			}

			Trajectory *cur = plan[planI];*/

			if(t >= actions[0][planI]->GetEndTime())
				planI++;
			if(planI >= actions[0].size()){
				state = STATE_LANDING;
				v.land();
				continue;
			}
			Trajectory *cur = static_cast<MotionAction*>(actions[0][planI])->GetPath();
			posctl.track(cur);
			posctl.control(t);
		}
		else if(state == STATE_LANDING) {
			//h2->control(0.0);
			// TODO:
		}

		i++;

		r.sleep();
	}



	// Cleanup
	if(sim_enabled) {
		gazebo->disconnect();
	}



	// Stop all vehicles
	v.disconnect();

	printf("Done!\n");


}
