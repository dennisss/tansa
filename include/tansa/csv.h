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

ActionTypes parse_action_type(const std::string& data){
	if(data == "transition")
		return ActionTypes::Transition;
	else if (data == "line")
		return ActionTypes::Line;
	else if (data == "circle")
		return ActionTypes::Circle;
	else if (data == "hover")
		return ActionTypes::Hover;
	else if (data == "light")
		return ActionTypes::Light;
	else if (data == "strobe")
		return ActionTypes::Strobe;
	return ActionTypes::None;
}
bool is_light_action(ActionTypes type){
	if (type == ActionTypes::Strobe || type == ActionTypes::Light)
		return true;
	return false;
}
std::vector<string> read_csv_line(std::string& line);
Choreography* parse_csv(const char* filepath);
double parse_time(std::string& time);
std::vector<unsigned long> parse_drones(std::string drone_field, const std::map<std::string, unsigned long>& drone_map);
Action* parse_motion_action(ActionTypes type, double start, double end, unsigned long droneid, std::vector<std::string> split_line);
LightAction* parse_light_action(ActionTypes type, double start, double end, unsigned long droneid, std::vector<std::string> split_line);
Action* parse_hover_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
Action* parse_line_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
Action* parse_circle_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
LightAction* parse_light_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);
LightAction* parse_strobe_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line);

}
#endif