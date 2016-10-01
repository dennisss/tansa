#include <tansa/core.h>
#include <tansa/time.h>
#include <tansa/vehicle.h>
#include <tansa/control.h>
#include <tansa/mocap.h>

#include <signal.h>
#include <unistd.h>

#include <vector>

/*
	Listen on a port 14550 for messages.
	When I get one, mark and recognize the MAV by it's port and ip address
	- Maintain its heartbeat status
	- Maintain the current mode of the vehicle
	- We should dedicate a single thread to sending and receiving from this MAV

	It will be responsible for setting new modes and sending continous
*/



/*
	Each MAV will consist of a UDP client/server for talking to it and a single thread for receiving things like heatbeats, etc.

	Everything else doesn't need to be on that thread as everything else just send mavlink messages
*/

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
		{2, 2, 2},
		{-2, 2, 2},
		{-2, -2, 2},
		{2, -2, 2},
		{0, 0, 2}
	};


	int pointI = 0;

	int i = 0;

	Vector3d integral(0,0,0);

	float level = 0;
	float dl = 0.005;


	PositionController posctl(&v);

	/*
		General procedure
		- Arm/takeoff to preset aerial home position
		- Execute the routine
		- Land/disarm at current position
	*/
	Time start(0,0);
	Rate r(100);

	while(running) {

		double t = Time::now().since(start).seconds();

/*
		usleep(10000);
		continue;


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



		if(pointI == 0) {
			if(pointI == points.size()){
				break;
			}

			double dist = (points[pointI] - v.position).norm();



			if(dist < 0.1) {
				start = Time::now();
				pointI++;

			}

			v.setpoint_pos(points[pointI]);


		}
		else {

			// Use Position controller
			posctl.control(t);

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
