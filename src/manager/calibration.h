#ifndef TANSA_MANAGER_CALIBRATION_H_
#define TANSA_MANAGER_CALIBRATION_H_

#include <tansa/vehicle.h>


namespace tansa {


enum CalibrationState {
	CalibrationInit = 0,
	CalibrationStarted,
	CalibrationFailed,
	CalibrationDone
};

/**
 * For performing calbration of drones
 * So far, this manages onboard gyro calibration of many drones while not flying
 */
class CalibrationHelper {
public:

	CalibrationHelper(const vector<Vehicle *> &vehicles);

	void step();

	bool done(); /**< Whether or not we are finished with trying to calibrate */
	bool failed(); /**< If done, this will  */

	vector<Vehicle *> vehicles;
	vector<CalibrationState> states;
	vector<double> percentage; /**< If the calibration has started, this is the percent completion */

private:

	void onTextMessage(const TextMessage &msg, unsigned idx);

	vector<Time> times;
	unsigned iterations; /**< Number of times the vehicles have been pinged */

};


}

#endif
