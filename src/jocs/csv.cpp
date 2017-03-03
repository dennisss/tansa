//
// Created by kyle on 2/28/17.
//

#include "tansa/csv.h"

#include <boost/algorithm/string.hpp>
namespace tansa {
std::stringstream read_whole_file(const char* filepath){
	std::ifstream t(filepath);
	std::stringstream buffer;
	return std::move(buffer);
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
			break;
		case ActionTypes::Line:
			break;
		case ActionTypes::Circle:
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
			break;
		case ActionTypes::Strobe:
			break;
		default:
			break;
	}
	return ret;

}


}
