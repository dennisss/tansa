#include "preview.h"

#include <iostream>
using namespace std;

namespace tansa {

PreviewPlayer::PreviewPlayer(Routine *data) {
	this->data = data;
	this->time = 0;
}

void PreviewPlayer::step() {
	if(!playing)
		return;

	this->time = rate*Time::now().since(this->playingStart).seconds() + this->playingStartTime;
}

bool PreviewPlayer::ended() {
	return this->time >= data->duration();
}


vector<ModelState> PreviewPlayer::get_states() {

	vector<ModelState> states;
	states.resize(data->actions.size());

	for(int i = 0; i < data->actions.size(); i++) {

		for(int j = 0; j < data->actions[i].size(); j++) {

			MotionAction *m = (MotionAction *) data->actions[i][j];
			Trajectory::Ptr path = m->GetPath();

			if(path->startTime() <= this->time && path->endTime() > this->time) {

				TrajectoryState ts = path->evaluate(this->time);

				states[i].position = ts.position;

				// TODO: This is redundant with PositionController
				Vector3d a = ts.acceleration + Vector3d(0, 0, GRAVITY_MS);
				Quaterniond att = Quaterniond::FromTwoVectors(Vector3d(0,0,1), a.normalized());
				states[i].orientation = att;

				break;
			}

		}

	}

	return states;
}

vector<int> PreviewPlayer::get_light_states() {

	vector<int> states;

	states.resize(data->lightActions.size(), 0); // If no action, default to off

	for(int i = 0; i < data->lightActions.size(); i++) {
		for(int j = 0; j < data->lightActions[i][0].size(); j++) {

			LightAction *m = data->lightActions[i][0][j];
			LightTrajectory::Ptr path = m->GetPath();

			if(path->getStartTime() <= this->time && path->getEndTime() > this->time) {
				int ts = path->evaluate(this->time);
				states[i] = ts;
				break;
			}
		}
	}


	return states;

}


}
