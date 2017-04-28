/*
	Bare minimum loop for testing vehicle functions on a single physical vehicle
	Currently set up for testing lighting
*/

#include <tansa/action.h>
#include <tansa/control.h>
#include <tansa/core.h>
#include <tansa/jocsParser.h>
#include <tansa/config.h>
#include <tansa/jocsPlayer.h>
#include <tansa/mocap.h>
#include <tansa/gazebo.h>
#include <tansa/osc.h>

#ifdef  __linux__
#include <sys/signal.h>
#endif
//TODO check if these work on OSX
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace tansa;


static bool running;

void signal_sigint(int s) {
	// TODO: Prevent
	running = false;
}

int main(int argc, char *argv[]) {
/*
	// TODO: Make this into a test

	double start = 661, end = 662;


	Trajectory *p = new LinearTrajectory({-3.5, 0, 0}, start, {-3.5, 0, 0}, end);

	cout << "- Traj: " << p->evaluate(start).position << endl;

	return 0;

*/

	Vehicle veh;
	veh.connect(14550 + 10*11, 14555, NULL, "192.168.1.92");


	signal(SIGINT, signal_sigint);
	running = true;
	printf("running...\n");


	int chan = 0;

	double i = 0.0;
	double dir = 1.0;

	Rate rate(100);
	while(running) {


		int r = 0, g = 0, b = 0;

		int v = 255 * i;

		if(chan == 0) {
			r = v;
		}
		else if(chan == 1) {
			g = v;
		}
		else if(chan == 2) {
			b = v;
		}
		else if(chan == 3) {
			r = v;
			g = v;
			b = v;
		}


//veh.set_beacon(true);
		/*
		if(dir > 0) {
			veh.set_beacon(false);
		}
		else {
			veh.set_beacon(true);
		}
		*/



		int c = b | (g << 8) | (r << 16);
		veh.set_lighting({ c, c, c });

		//cout << veh.lightState[0] << " " << veh.lightState[1] << " " << veh.lightState[2] << endl;


		i = i + dir * 0.01;

		if(i >= 1.0 || i < 0) {
			dir = -dir;

			if(dir == 1.0) {
				chan = (chan + 1) % 4;
			}
		}

		rate.sleep();
	}


	printf("Done!\n");

}
