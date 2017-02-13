
#include <tansa/simulate.h>


using namespace tansa;


int main(int argc, char *argv[]) {

	World w;
	WorldState ws;

	int n = 1;

	ws.models.resize(n);

	DataObject desc = DataObject::LoadFile("config/models/x260.js");

	// Add all drones to the world
	for(int i = 0; i < n; i++) {
		MultirotorModel::Ptr m( new MultirotorModel(desc) );
		w.models.push_back(m);

		State::Ptr s = m->defaultState();

		ws.models.push_back(s);

//		m->state.position = Vector3d(1,2,0);
	}

	Simulation sim(w, ws);
	// TODO: This should run run in a separate thread
	// It should also start up the Firmwares for the drones and be ready to communicate
	sim.step();



}
