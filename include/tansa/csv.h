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
enum csv_positions : uint32_t {
	BreakpointPos = 0,
	StartTimePos = 1,
	EndTimePos = 2,
	ActionTypePos = 3,
	DronesPos = 4,
	ParamStartPos = 5,
	DroneKeyPos = 7,
};
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
 * @param data
 * @return
 */
ActionTypes parse_action_type(const std::string& data);
/**
 *
 * @param type
 * @return
 */
bool is_light_action(ActionTypes type);
/**
 *
 * @param line
 * @return
 */
std::vector<string> read_csv_line(std::string& line);
/**
 *
 * @param filepath
 * @return
 */
Choreography* parse_csv(const char* filepath);
/**
 *
 * @param time
 * @return
 */
double parse_time(std::string& time);
/**
 *
 * @param drone_field
 * @param drone_map
 * @return
 */
std::vector<unsigned long> parse_drones(std::string drone_field, const std::map<std::string, unsigned long>& drone_map);
/**
 *
 * @param type
 * @param start
 * @param end
 * @param droneid
 * @param split_line
 * @return
 */
Action* parse_motion_action(ActionTypes type, double start, double end, unsigned long droneid, std::vector<std::string> split_line);
/**
 *
 * @param type
 * @param start
 * @param end
 * @param droneid
 * @param split_line
 * @return
 */
LightAction* parse_light_action(ActionTypes type, double start, double end, unsigned long droneid, std::vector<std::string> split_line);
/**
 *
 * @param type
 * @param start
 * @param end
 * @param droneid
 * @param split_line
 * @return
 */
Action* parse_hover_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
/**
 *
 * @param type
 * @param start
 * @param end
 * @param droneid
 * @param split_line
 * @return
 */
Action* parse_line_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
/**
 *
 * @param type
 * @param start
 * @param end
 * @param droneid
 * @param split_line
 * @return
 */
Action* parse_circle_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
/**
 *
 * @param type
 * @param start
 * @param end
 * @param droneid
 * @param split_line
 * @return
 */
LightAction* parse_light_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
/**
 *
 * @param type
 * @param start
 * @param end
 * @param droneid
 * @param split_line
 * @return
 */
LightAction* parse_strobe_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
/**
 * NOTE: This also sorts each sub array of actions by start time.
 * @param actions
 * @param homes
 * @return
 */
bool has_no_discontinuity(std::vector<std::vector<Action*>>& actions, const std::vector<Point>& homes);
/**
 *
 * @param actions
 */
void insert_transitions(std::vector<std::vector<Action*>>& actions, const std::vector<Point>& homes);
/**
 *
 * @param point
 * @return
 */
Point parse_point(std::string point);
}
#endif