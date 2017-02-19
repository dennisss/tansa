
#include <tansa/simulate.h>

#include <unistd.h>
#include <signal.h>

using namespace std;
using namespace tansa;


static bool running;

void signal_sigint(int s) {
	// TODO: Prevent
	running = false;
}


int main(int argc, char *argv[]) {

	tansa::init(true);

	Simulation *s = Simulation::Make();
	s->start();

	signal(SIGINT, signal_sigint);
	running = true;
	printf("running...\n");

	unsigned i = 0;
	Rate r(10000);
	while(running) {
		usleep(100000);
	}

	s->stop();
	delete s;

	//cout << "T: " << sim.state.time.seconds() << endl;

	printf("Done!\n");

}
