#include <tansa/simulate.h>

#include <signal.h>
#include <unistd.h>


namespace tansa {


Firmware::Firmware(MultirotorModel::Ptr model) {
	currentActuatorOutputs.resize(4, 0);

	model->imu->subscribe(&Firmware::onImuData, this);
}


void Firmware::start() {
	int p = fork();
	if(p == 0) { // Child
		char *const bash = (char *const) "/bin/bash";
		char *const script = (char *const) "scripts/start_instance.sh";
		char *const rc_script = (char *const) this->rcScript.c_str(); // "config/gazebo/x340";
		char num[16];
		strcpy(num, std::to_string(this->id).c_str());

		char *const argv[] = { bash, script, num, rc_script, NULL};

		execv(bash, argv);

		exit(0);
		return;
	}

	this->process = p;

	this->sim_vehicle = new Vehicle();
	this->sim_vehicle->connect(14561 + 10*id, 0);
	this->sim_vehicle->subscribe(&Firmware::onActuatorOutputs, this);
}

void Firmware::stop() {
	if(process == 0)
		return;

	kill(process, SIGINT);
	waitpid(process, NULL, 0);

}

void Firmware::connectClient(Vehicle *v) {
	// These should match 'scripts/start_instance.sh'
	int lport = 14550 + 10*id,
		rport = 14555 + 10*id;

	v->connect(lport, rport);
}

void Firmware::onImuData(const IMUSensorData *data) {
	sim_vehicle->hil_sensor(&data->accel, &data->gyro, NULL, data->time);
}

void Firmware::onActuatorOutputs(const ActuatorOutputs *actuators) {
	currentActuatorOutputs.resize(4);
	for(int i = 0; i < 4; i++) {
	 	currentActuatorOutputs[i] = actuators->outputs[i];
	}
}


}
