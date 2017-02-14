
#include <tansa/simulate.h>

#include <signal.h>

using namespace std;
using namespace tansa;


static bool running;

void signal_sigint(int s) {
	// TODO: Prevent
	running = false;
}


int main(int argc, char *argv[]) {

	World w;
	WorldState ws;
	vector<shared_ptr<Firmware>> firmwares;

	int n = 1;

	ws.models.resize(n);

	DataObject desc = DataObject::LoadFile("config/models/x260.js");

	// Add all drones to the world
	for(int i = 0; i < n; i++) {
		MultirotorModel::Ptr m( new MultirotorModel(desc) );
		w.models.push_back(m);

		firmwares.push_back( shared_ptr<Firmware>( new Firmware(desc["firmware"], m) ) );

		firmwares[i]->start();

		State::Ptr s = m->defaultState();

		ws.models[i] = s;

//		m->state.position = Vector3d(1,2,0);
	}

	Simulation sim(w, ws);
	// TODO: This should run run in a separate thread
	// It should also start up the Firmwares for the drones and be ready to communicate

	std::shared_ptr<MultirotorModelState> s = std::static_pointer_cast<MultirotorModelState>(sim.state.models[0]);
	cout << s->position << endl;


	signal(SIGINT, signal_sigint);
	running = true;
	printf("running...\n");

	Rate r(10000);
	while(running) {
		firmwares[0]->update(sim.state.models[0]);
		sim.step();
//		cout << s->position.transpose() << endl;
		r.sleep();
	}

	cout << s->position << endl;
	cout << "T: " << sim.state.time.seconds() << endl;


	printf("Done!\n");


}
