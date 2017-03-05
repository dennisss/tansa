#ifndef TANSA_CSV_H
#define TANSA_CSV_H
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <iterator>
#include "tansa/action.h"
#include "tansa/vehicle.h"
#include "tansa/core.h"
#include "tansa/trajectory.h"
#include "tansa/action.h"
#include "tansa/breakpoint.h"
namespace  tansa {

const double FEET_TO_METERS = 0.3048;
const double DEGREES_TO_RADIANS = M_PI/180.0;
/**
 * Named constants for the columns in the CSV which are used for specific purposes
 */
enum csv_positions : uint32_t {
	BreakpointPos = 0,
	StartTimePos = 1,
	EndTimePos = 2,
	ActionTypePos = 3,
	DronesPos = 4,
	ParamStartPos = 5,
	LengthUnitPos = 6,
	AngleMeasurePos = 7,
	DroneKeyPos = 9,
};
/**
 * @struct
 * Holds all the information necessary to run a choreography.
 * Cleans up all resources when freed.
 */
struct Choreography {
	~Choreography() {
		for(const auto& d: actions){
			for(const auto& a: d){
				delete a;
			}
		}
		for(const auto& d: lightActions){
			for(const auto& a: d){
				delete a;
			}
		}
	}
	std::vector<std::vector<Action*>> actions;
	std::vector<Breakpoint> breakpoints;
	std::vector<Point> homes;
	std::vector<std::vector<LightAction*>> lightActions;
	bool needConvertToMeters = false;
	bool needConvertToRadians = false;
};
/**
 *
 * @param data A string containing a string representation of an action
 * @return The enum version of passed in string action.
 */
ActionTypes parse_action_type(const std::string& data);
/**
 *
 * @param type The type of action
 * @return Whether or not the action is related to lighting.
 */
bool is_light_action(ActionTypes type);
/**
 *
 * @param line A line in a csv file
 * @return A vector of strings representing each cell of passed in line
 */
std::vector<string> read_csv_line(std::string& line);
/**
 * NOTE: Returns nullptr if file is invalid/has discontinuities
 * @param filepath Path to csv file
 * @return A choreography object representing data in that file.
 */
Choreography* parse_csv(const char* filepath, double scale);
/**
 *
 * @param time Given a time as a string (format SS.MS)
 * @return Double precision floating point representation of the given time string in seconds.
 */
double parse_time(std::string& time);
/**
 *
 * @param drone_field A string filed of the form (Drone1;Drone2; etc... )
 * @param drone_map Map of drone name to DroneId
 * @return
 */
std::vector<unsigned long> parse_drones(std::string drone_field, const std::map<std::string, unsigned long>& drone_map);
/**
 *
 * @param type Type of action
 * @param start Start time in seconds
 * @param end End time in seconds
 * @param droneid Numeric id of the drone.
 * @param split_line Split CSV line from parse_csv_line
 * @param length_conversion Conversion factor for length
 * @param angle_conversion Conversion factor for angle
 * @return The parsed action for the given drone.
 */
Action* parse_motion_action(ActionTypes type,
							double start,
							double end,
							unsigned long droneid,
							std::vector<std::string> split_line,
							double length_conversion = 1.0,
							double angle_conversion = 1.0);
/**
 *
 * @param type Type of action
 * @param start Start time in seconds
 * @param end End time in seconds
 * @param droneid Numeric id of the drone.
 * @param split_line Split CSV line from parse_csv_line
 * @return The parsed action for the given drone.
 */
LightAction* parse_light_action(ActionTypes type, double start, double end, unsigned long droneid, std::vector<std::string> split_line);
/**
 *
 * @param type Type of action
 * @param start Start time in seconds
 * @param end End time in seconds
 * @param droneid Numeric id of the drone.
 * @param split_line Split CSV line from parse_csv_line
 * @param length_conversion Conversion factor for length
 * @return The parsed HoverAction for the given drone.
 */
Action* parse_hover_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion);
/**
 *
 * @param type Type of action
 * @param start Start time in seconds
 * @param end End time in seconds
 * @param droneid Numeric id of the drone.
 * @param split_line Split CSV line from parse_csv_line
 * @param length_conversion Conversion factor for length
 * @return The parsed LineAction for the given drone.
 */
Action* parse_line_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion);
/**
 *
 * @param type Type of action
 * @param start Start time in seconds
 * @param end End time in seconds
 * @param droneid Numeric id of the drone.
 * @param split_line Split CSV line from parse_csv_line
 * @param length_conversion Conversion factor for length
 * @param angle_conversion Conversion factor for angle
 * @return The parsed CircleAction for the given drone.
 */
Action* parse_circle_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion, double angle_conversion);
/**
 *
 * @param type Type of action
 * @param start Start time in seconds
 * @param end End time in seconds
 * @param droneid Numeric id of the drone.
 * @param split_line Split CSV line from parse_csv_line
 * @return The parsed LightAction for the given drone.
 */
LightAction* parse_simple_light_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
/**
 *
 * @param type Type of action
 * @param start Start time in seconds
 * @param end End time in seconds
 * @param droneid Numeric id of the drone.
 * @param split_line Split CSV line from parse_csv_line
 * @return The parsed StrobeAction for the given drone.
 */
LightAction* parse_strobe_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
/**
 * NOTE: This also sorts each sub array of actions by start time.
 * @param actions 2D array of action pointers
 * @param homes Starting points of the drones
 * @return True if no discontinuities are found false otherwise
 */
bool has_no_discontinuity(std::vector<std::vector<Action*>>& actions, const std::vector<Point>& homes);
/**
 * Replaces all empty actions with transitions
 * @param actions 2D array of action pointers
 */
void insert_transitions(std::vector<std::vector<Action*>>& actions, const std::vector<Point>& homes);
/**
 *
 * @param point A string representing a point
 * @return The tansa::point representation parse from a string.
 */
Point parse_point(std::string point);
}
#endif