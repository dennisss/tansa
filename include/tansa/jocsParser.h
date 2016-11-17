#ifndef TANSA_JOCSPARSER_H
#define TANSA_JOCSPARSER_H
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>
#include "json.hpp"
#include "tansa/vehicle.h"
#include "tansa/core.h"
#include "tansa/trajectory.h"
#include "tansa/action.h"
namespace tansa {
/**
 * Class representing the
 */
class Jocs {
public:
	static const std::string HOME_KEY;
	static const std::string ID_KEY;
	static const std::string CHOREOGRAPHY_KEY;
	static const std::string STARTPOS_KEY;
	static const std::string ENDPOS_KEY;
	static const std::string DURATION_KEY;
	static const std::string ACTION_ROOT_KEY;
	static const std::string ACTION_TIME_KEY;
	static const std::string ACTION_TYPE_KEY;
	static const std::string DRONE_ARRAY_KEY;
	static const std::string ACTION_DATA_KEY;
	static const std::string CIRCLE_ORIGIN_KEY;
	static const std::string CIRCLE_RADIUS_KEY;
	static const std::string CIRCLE_THETA1_KEY;
	static const std::string CIRCLE_THETA2_KEY;
	static const std::string UNITS_KEY;
	static const double FEET_TO_METERS;
	static const double DEGREES_TO_RADIANS;
	static const std::string THEATER_VOLUME_KEY;
	static const std::string MIN_KEY;
	static const std::string MAX_KEY;
	static const std::string LENGTH_KEY;
	static const std::string TIME_KEY;
	static const std::string ANGLE_KEY;
	static const std::string REPEAT_KEY;
	static const std::string DRONE_START_OFF_KEY;
	static const std::string DRONE_END_OFF_KEY;
	static const std::string HOVER_KEY;
	static const std::string DRONE_OFF_KEY;
    static const std::string BREAK_KEY;
    static const std::string BREAK_NAME;
    static const std::string BREAK_START;
    static const std::string BREAK_NUMBER;

	Jocs(bool convertMeters, bool convertRadians, unsigned numRepeat) : needConvertToMeters(convertMeters), needConvertToRadians(convertRadians), repeat(numRepeat){}
	~Jocs();
	/**
	 * Parses a given Jocs file
	 * @public
	 * @param jocsPath String containing path from working directory to jocsFile
	 * @return A vector containing unique_ptrs to Actions
	 */
	static Jocs Parse(std::string jocsPath, double scale);
	inline std::vector<Point> GetHomes() const { return homes; }
	inline const std::vector<std::vector<Action*>>& GetActions() const { return actions; }

	double getNextBreakpointTime(double lastTime);
	double getBreakpointTime(unsigned breakpointNumber);
	double getBreakpointTime(std::string breakpointName);
	unsigned getBreakpointNumber(double startTime);
	Point getDroneLocationAtTime(double startTime, unsigned droneId);

private:
	bool needConvertToMeters = false;
	bool needConvertToRadians = false;
	std::vector<Point> homes;
	std::vector<std::vector<Action*>> actions;
	unsigned repeat;

private:
	/**
	 * Parse all actions in a given Jocs json object
	 * @param data [in] Jocs json data
	 * @param actions [out] Will be filled with actions
	 */
	 void parseActions(const nlohmann::json &data, double scale);
	/**
	 * Parses a singular action array
	 * @private
	 * @param data Json data containing a reference to an action
	 * @param actions [out] Will be filled with actions
	 */
	 double parseAction(nlohmann::json::reference data, double lastTime, double scale);
	/**
	 * Converts from string to ActionTypes enum
	 * @private
	 * @param data A string containing the ascii representation of an ActionType
	 * @return Action type as the enum
	 */
	 ActionTypes convertToActionType(const std::string& data);
};
}
#endif //TANSA_JOCSPARSER_H
