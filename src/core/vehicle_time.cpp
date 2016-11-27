#include <tansa/vehicle.h>
#include <tansa/time.h>

/*
	For vehicle specific time synchronization

	This file is a derivative of the corresponding source code in the PX4 Firmware (see file 'src/modules/mavlink/mavlink_receiver.cpp'). In particular, the implementation for 'timesync' related functions.

	A direct link to the reference version is here: https://github.com/PX4/Firmware/blob/660a9e13e7649955ffec8e4956bd83c2cfa0212a/src/modules/mavlink/mavlink_receiver.cpp

	This file should continue to mirror that implementation to maintain compatibility with the synchronization protocol
*/

namespace tansa {

// Given a time received from this Vehicle, return it in this computer's time frame
uint64_t Vehicle::sync_stamp(uint64_t usec) {

	if(_time_offset != 0) {
		return usec + (_time_offset / 1000) ;

	}
	else {
		return Time::now().micros();
	}
}


void Vehicle::smooth_time_offset(int64_t offset_ns) {
	/* alpha = 0.6 fixed for now. The closer alpha is to 1.0,
	 * the faster the moving average updates in response to
	 * new offset samples.
	 */
	 double _time_offset_avg_alpha = 0.6;

	_time_offset = (_time_offset_avg_alpha * offset_ns) + (1.0 - _time_offset_avg_alpha) * _time_offset;
}

void Vehicle::handle_message_timesync(mavlink_message_t *msg) {

	mavlink_timesync_t tsync;
	mavlink_msg_timesync_decode(msg, &tsync);

	uint64_t now_ns = Time::now().nanos();

	// We received a request to be synchronized
	if(tsync.tc1 == 0) {
		send_timesync(now_ns, tsync.ts1);
		return;
	}
	// We received a response to our timesync request
	else if(tsync.tc1 > 0) {

		int64_t offset_ns = (int64_t)(tsync.ts1 + now_ns - tsync.tc1 * 2) / 2 ;
		int64_t dt = _time_offset - offset_ns;

		if (dt > 10000000LL || dt < -10000000LL) { // 10 millisecond skew
			_time_offset = offset_ns;
			//printf("[timesync] Hard setting offset.\n");

		} else {
			smooth_time_offset(offset_ns);
		}
	}

}

// This sends the real calendar time to the MAV in milliseconds (mainly used for logging with the right time)
void Vehicle::send_systime() {

	Time t = Time::now();

	int64_t mt = t.micros();
	int64_t mt_boot = t.sinceStart().millis();

	mavlink_message_t msg;
	mavlink_msg_system_time_pack(
		255, 0,
		&msg,
		mt,
		mt_boot
	);

	send_message(&msg);

}

// Both are in nanoseconds
void Vehicle::send_timesync(int64_t tc1, int64_t ts1) {

	mavlink_message_t msg;
	mavlink_msg_timesync_pack(
		255, 0,
		&msg,
		tc1,
		ts1
	);

	send_message(&msg);
}

}
