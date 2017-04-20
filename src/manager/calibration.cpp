#include "calibration.h"

#include <functional>
#include <regex>
#include <iostream>

using namespace std;


namespace tansa {

CalibrationHelper::CalibrationHelper(const vector<Vehicle *> &vehicles) {


	this->vehicles = vehicles;

	for(unsigned i = 0; i < vehicles.size(); i++) {
		auto callback = std::bind(&CalibrationHelper::onTextMessage, std::placeholders::_2, std::placeholders::_1, i);
		//vehicles[i]->subscribe(callback, this);
		vehicles[i]->calibrate_gyro();
	}

	states.resize(vehicles.size(), CalibrationInit);
	times.resize(vehicles.size(), Time::now());
//	iterations.resize(vehicles.size(), 1);
	percentage.resize(vehicles.size(), 0.0);
}


bool CalibrationHelper::done() {
	for(auto s : states) {
		if(s != CalibrationFailed && s != CalibrationDone) {
			return false;
		}
	}

	return true;
}

bool CalibrationHelper::failed() {
	for(auto s : states) {
		if(s == CalibrationFailed) {
			return true;
		}
	}

	return false;
}

void CalibrationHelper::step() {

	Time t = Time::now();

	for(unsigned i = 0; i < vehicles.size(); i++) {

		// Process all messages
		TextMessage m;
		while(vehicles[i]->get_message(&m)) {
			this->onTextMessage(m, i);
		}

		double dt = t.since(times[i]).seconds();

		if(states[i] == CalibrationInit) {

			// After a timeout, re-send
			// After a large timeout, fail it

			if(dt > 3) {
				states[i] = CalibrationFailed;
			}

		}
		else if(states[i] == CalibrationStarted) {
			if(dt > 3) { // Check for message timeout
				states[i] = CalibrationFailed;
			}
		}

	}


}

// TODO: Either queue the text messages or mutex lock the times array to do this right
void CalibrationHelper::onTextMessage(const TextMessage &msg, unsigned int idx) {

	string txt = msg.text;

	// Filter non-calibration messages
	if(txt.find("[cal]") != 0) {
		return;
	}


	smatch m;
	regex re_started("\\[cal\\] calibration started.*");
	regex re_progress("\\[cal\\] progress <([0-9]+)>");
	regex re_done("\\[cal\\] calibration done.*");
	regex re_failed("\\[cal\\] calibration (failed|cancelled).*");

	if(regex_match(txt, m, re_started)) {
		states[idx] = CalibrationStarted;

	}
	else if(regex_match(txt, m, re_progress)) {
		int percent = stoi(m[1].str());
		percentage[idx] = percent;

		if(states[idx] == CalibrationInit) {
			states[idx] = CalibrationStarted;
		}

		cout << "Got: " << percent << endl;
	}
	else if(regex_match(txt, m, re_done)) {
		states[idx] = CalibrationDone;
	}
	else if(regex_match(txt, m, re_failed)) {
		states[idx] = CalibrationFailed;
	}
	else {
		return;
	}

	times[idx] = Time::now();
}


}
