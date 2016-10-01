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

int main(int argc, char *argv[]) {

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
		tansa::sim_track(&v);
	}
	if(mocap_enabled) {
		string client_addr = "192.168.2.1";
		mocap = new Mocap();
		mocap->connect(client_addr);
		mocap->track(&v, 1);
	}


	running = true;
	signal(SIGINT, signal_sigint);

	vector<Vector3d> points = {
		{0, 0, 1},
		{2, 2, 1},
		{-2, 2, 1},
		{-2, -2, 1},
		{2, -2, 1},
		{0, 0, 1}
	};


	#define STATE_TAKEOFF 0
	#define STATE_FLYING 1
	#define STATE_LANDING 2

	int state = STATE_TAKEOFF;

	int i = 0;

	float level = 0;
	float dl = 0.005;

	HoverController hover(&v, points[0]);
	PositionController posctl(&v);

	//CircleTrajectory circle;
	Trajectory *square[5];
	for(int i = 0; i < 5; i++) {
		square[i] = new LinearTrajectory(points[i], 10.0*i, points[i+1], 10.0*(i+1));
	}


	Time start(0,0);
	Rate r(100);

	while(running) {

		double t = Time::now().since(start).seconds();

/*
		Sample Lighting stuff

		v.set_lighting(level, level);

		level += dl;
		if(level >= 1.0 || level <= 0.0)
			dl = -dl;
*/


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



		if(state == STATE_TAKEOFF) {

			hover.control(t);

			if(hover.distance() < 0.1) {
				start = Time::now();
				state = STATE_FLYING;
			}

		}
		else if(state == STATE_FLYING) {

			int idx = (int)floor(t / 10.0);

			if(idx >= (sizeof(square) / sizeof(Trajectory *))) {
				state = STATE_LANDING;
				v.land();
				continue;
			}

			Trajectory *cur = square[idx];
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
