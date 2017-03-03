//
// Created by kyle on 2/28/17.
//

#include "tansa/csv.h"

#include <boost/algorithm/string.hpp>
namespace tansa {

template<typename Out>
void split(const std::string &s, char delim, Out result) {
	std::stringstream ss;
	ss.str(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		*(result++) = item;
	}
}


std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, std::back_inserter(elems));
	return elems;
}

std::vector<string> read_csv_line(std::string& line){
	auto ret = split(line, ',');
	return std::move(ret);

}
Choreography* parse_csv(const char* filepath){
	auto csv = ifstream(filepath);
	auto ret = new Choreography();
	auto drone_map = std::map<std::string, unsigned long>();
	std::string line;
	getline(csv, line); //Contains dronekey
	std::vector<std::string> split_line;
	split_line = split(line, ',');
	unsigned break_num = 0;
	//TODO: Parse Dronekey
	int start_index = csv_positions::DroneKeyPos;
	unsigned long num_drones = 0;
	for (int i = start_index; i < split_line.size(); i++){
		if(split_line[i].empty())
			break;
		drone_map[split_line[i]] = num_drones;
		num_drones++;
	}
	ret->actions.resize(num_drones);
	ret->lightActions.resize(num_drones);
	//TODO: Parse homes
	while(getline(csv,line)){ //Iterate line by line
		split_line = std::move(read_csv_line(line)); //TODO: Probably inefficient.

		//Parse breakpoints
		if(!split_line[csv_positions::BreakpointPos].empty()){
			ret->breakpoints.push_back(
					Breakpoint(split_line[csv_positions::BreakpointPos],
							   break_num++,
							   parse_time(split_line[csv_positions::StartTimePos])));
		}

		double start_time = parse_time(split_line[csv_positions::StartTimePos]);
		double end_time = parse_time(split_line[csv_positions::EndTimePos]);
		ActionTypes action_type = parse_action_type(split_line[csv_positions::ActionTypePos]);
		std::vector<unsigned long> drones = parse_drones(split_line[csv_positions::DronesPos], drone_map);
		if(is_light_action(action_type)) {
			for(int i = 0; i < drones.size(); i++){
				auto light_action = parse_light_action(
						action_type,
						start_time,
						end_time,
						drones[i],
						std::vector<std::string>(split_line.begin() + csv_positions::ParamStartPos, split_line.end()));
				ret->lightActions[drones[i]].push_back(light_action);
			}
		} else{
			for(int i = 0; i < drones.size(); i++) {
				auto motion_action = parse_motion_action(
						action_type,
						start_time,
						end_time,
						drones[i],
						std::vector<std::string>(split_line.begin() + csv_positions::ParamStartPos, split_line.end()));
				ret->actions[drones[i]].push_back(motion_action);
			}
		}
	}
	std::cout << "Done!" << std::endl;

}

double parse_time(std::string& time){
	return std::atof(time.c_str());
}

std::vector<unsigned long> parse_drones(std::string drone_field, const std::map<std::string, unsigned long>& drone_map) {
	boost::erase_all(drone_field, "(");
	boost::erase_all(drone_field, ")");
	boost::erase_all(drone_field, "\"");
	boost::erase_all(drone_field, " ");
	auto split_line = split(drone_field, ';');
	std::vector<unsigned long> ret;
	for(std::string s : split_line){
		unsigned long i = drone_map.at(s);
		ret.push_back(i);
	}
	return std::move(ret);
}

Action* parse_motion_action(ActionTypes type, double start, double end, unsigned long droneid, std::vector<std::string> split_line) {
	Action* ret = nullptr;
	switch (type){
		case ActionTypes::Hover:
			ret = parse_hover_action(start, end, droneid, split_line);
			break;
		case ActionTypes::Line:
			ret = parse_line_action(start, end, droneid, split_line);
			break;
		case ActionTypes::Circle:
			ret = parse_circle_action(start, end, droneid, split_line);
			break;
		case ActionTypes::Transition:
			ret = new EmptyAction((unsigned)droneid, start, end); //Transitions replaced later.
			break;
		case ActionTypes::None:
			break;
		default:
			ret = nullptr;
			break;
	}
	return ret;
}
LightAction* parse_light_action(ActionTypes type, double start, double end, unsigned long droneid, std::vector<std::string> split_line){
	LightAction* ret = nullptr;
	switch (type){
		case ActionTypes::Light:
			ret = parse_light_action(start, end, droneid, split_line);
			break;
		case ActionTypes::Strobe:
			ret = parse_strobe_action(start, end, droneid, split_line);
			break;
		default:
			ret = nullptr;
			break;
	}
	return ret;
}
bool is_number(const std::string& s) {
	return !s.empty() && std::find_if(s.begin(),
									  s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
}
Action* parse_hover_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line) {

	Point hover;
	hover.x() = std::atof(split_line[1].c_str());
	hover.y() = std::atof(split_line[3].c_str());
	hover.z() = std::atof(split_line[5].c_str());
	return new MotionAction(
					(unsigned) droneid,
					new LinearTrajectory(hover, start, hover, end),
					ActionTypes::Hover);

}

Action* parse_line_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line) {
	Point start_point, end_point;
	int j = 0;
	for(int i = 0; i<split_line.size();i++){
		if(!split_line[i].empty()){
			if(is_number(split_line[i])){
				if(j > 2){
					end_point[j%3] = std::atof(split_line[i].c_str());
				} else {
					start_point[j] = std::atof(split_line[i].c_str());
				}
				j++;
			}
		}
	}
	return new MotionAction(
			(unsigned) droneid,
			new LinearTrajectory(start_point, start, end_point, end),
			ActionTypes::Line);
}
Action* parse_circle_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line) {
	Point origin;
	double theta1 = std::atof(split_line[3].c_str());
	double theta2 = std::atof(split_line[5].c_str());
	double radius = std::atof(split_line[1].c_str());
	origin.x() = std::atof(split_line[7].c_str());
	origin.y() = std::atof(split_line[9].c_str());
	origin.z() = std::atof(split_line[11].c_str());
	return new MotionAction(
			(unsigned) droneid,
			new CircleTrajectory(origin, radius,theta1, start, theta2, end),
			ActionTypes::Circle);

}

LightAction* parse_light_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line){


}
LightAction* parse_strobe_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line){


}

}
