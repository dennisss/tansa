
#include <tansa/vehicle.h>
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

	tansa::init();

	signal(SIGINT, signal_sigint);

	Mocap *mocap = NULL;

	Vehicle v;
	v.connect();


	if(start_mocap) {
		string client_addr = "192.168.2.1";
		mocap = new Mocap();
		mocap->connect(client_addr);
		mocap->track(&v, 1);
	}



	running = true;

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
	double t = 0;

	Vector3d integral(0,0,0);


	while(running) {


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
			double dist = (points[pointI] - v.position).norm();

			if(dist < 0.1)
				pointI++;

			v.setpoint_pos(points[pointI]);
		}
		else {

			// Circle trajectory
			double r = 2;
			double dTheta = 18.0 * M_PI / 180.0; // radians per second
			double theta = t * dTheta; // angle as a function of time
			Vector3d pT(r*sin(theta), r*cos(theta), 1 + t / 10.0);
			Vector3d vT(r*dTheta*cos(theta), -r*dTheta*sin(theta), 1.0 / 10.0);
			Vector3d aT(-r*dTheta*dTheta*sin(theta), -r*dTheta*dTheta*cos(theta), 0);

			Vector3d eP = pT - v.position;
			Vector3d eV = vT - v.velocity;

			// PD gains
			Vector3d Kp(1.0, 1.0, 1.0);
			Vector3d Kd(0.1, 0.1, 0.3);
			Vector3d Ki(0, 0, 0);

			Vector3d a = Kp.cwiseProduct(eP) + Kd.cwiseProduct(eV) + Ki.cwiseProduct(integral);
			a = a / (9.8 / 0.5) + Vector3d(0, 0, 0.5);
			//a.x() = 0;
			//a.y() = 0;
			printf("A: %.2f %.2f %.2f\n", a.x(), a.y(), a.z());
			v.setpoint_accel(a);

			integral = integral + eP;

		}

	//	send_setpoint();
	//	usleep(100);

	//	if(i++ % 10 == 0) {
	//		send_heartbeat();
	//		if(i < 500)
	//			send_set_mode();
	//		else
	//			send_arm();
	//	}

		i++;
		t += 0.01;
		usleep(10000); // 100hz
	}

	// Stop all vehicles
	v.stop();

	printf("Done!\n");


}
