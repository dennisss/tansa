#ifndef TANSA_PREVIEW_H_
#define TANSA_PREVIEW_H_

#include <tansa/routine.h>
#include <tansa/time.h>
#include <tansa/model.h>

namespace tansa {

/**
 * Allows going through a quick idealized preview of the whole routine
 */
class PreviewPlayer {
public:

	PreviewPlayer(Routine *data);

	void step();

	void play() { this->playing = true; playingStart = Time::now(); playingStartTime = this->time; }
	void pause() { this->playing = false; }
	bool ended();

	bool is_playing() { return this->playing; }

	void set_rate(double r) { this->rate = r; playingStart = Time::now(); playingStartTime = this->time; }
	void set_time(double t) { this->time = t; playingStart = Time::now(); playingStartTime = this->time; }
	double get_time() { return this->time; }


	vector<ModelState> get_states();

private:

	Routine *data = NULL;
	double time = 0;
	double rate = 1;

	bool playing = false;
	Time playingStart;
	double playingStartTime;

};


}

#endif
