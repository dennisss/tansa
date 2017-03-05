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
	return type == ActionTypes::Strobe || type == ActionTypes::Light;
}

std::vector<string> read_csv_line(std::string& line){
	auto ret = split(line, ',');
	return std::move(ret);

}
Choreography* parse_csv(const char* filepath, double scale){
	Choreography* ret = nullptr;
	try {
		auto csv = ifstream(filepath);
		ret = new Choreography();
		auto drone_map = std::map<std::string, unsigned long>();
		std::string line;
		getline(csv, line); //Contains dronekey and meters/feet
		std::vector<std::string> split_line;
		split_line = split(line, ',');
		unsigned break_num = 0;
		double conversion_factor = scale;
		double angle_conversion = 1.0;
		std::string length_units = split_line[csv_positions::LengthUnitPos];
		std::string angle_units = split_line[csv_positions::AngleMeasurePos];
		ret->needConvertToMeters = length_units != "meters" && length_units != "Meters";
		ret->needConvertToRadians = angle_units != "radians" && angle_units != "Radians";
		if(ret->needConvertToMeters) {
			conversion_factor *= FEET_TO_METERS;
		}
		if(ret->needConvertToRadians) {
			angle_conversion *= DEGREES_TO_RADIANS;
		}
		int start_index = csv_positions::DroneKeyPos;
		unsigned long num_drones = 0;
		for (int i = start_index; i < split_line.size(); i++) {
			if (split_line[i].empty())
				break;
			drone_map[split_line[i]] = num_drones;
			num_drones++;
		}
		ret->actions.resize(num_drones);
		ret->lightActions.resize(num_drones);
		getline(csv, line); //contains homes
		auto home_split = split(line, ',');
		for (int i = csv_positions::ParamStartPos; i < home_split.size(); i++) {
			ret->homes.push_back(parse_point(home_split[i])*conversion_factor);
			if (ret->homes.size() == num_drones)
				break;
		}

		//TODO: Parse homes
		while (getline(csv, line)) { //Iterate line by line
			split_line = std::move(read_csv_line(line)); //TODO: Probably inefficient.

			//Parse breakpoints
			if (!split_line[csv_positions::BreakpointPos].empty()) {
				ret->breakpoints.push_back(
						Breakpoint(split_line[csv_positions::BreakpointPos],
								   break_num++,
								   parse_time(split_line[csv_positions::StartTimePos])));
			}

			double start_time = parse_time(split_line[csv_positions::StartTimePos]);
			double end_time = parse_time(split_line[csv_positions::EndTimePos]);
			ActionTypes action_type = parse_action_type(split_line[csv_positions::ActionTypePos]);
			std::vector<unsigned long> drones = parse_drones(split_line[csv_positions::DronesPos], drone_map);
			//currently only supporting one drone per line so just parse that one.
			if (is_light_action(action_type)) {
				for (int j = 0; j < 1; j++) {
					auto light_action = parse_light_action(
							action_type,
							start_time,
							end_time,
							drones[j],
							std::vector<std::string>(split_line.begin() + csv_positions::ParamStartPos,
													 split_line.end()));
					ret->lightActions[drones[j]].push_back(light_action);
				}
			} else {
				for (int j = 0; j < 1; j++) {
					auto motion_action = parse_motion_action(
							action_type,
							start_time,
							end_time,
							drones[j],
							std::vector<std::string>(split_line.begin() + csv_positions::ParamStartPos,
													 split_line.end()),
							conversion_factor,
							angle_conversion
					);
					ret->actions[drones[j]].push_back(motion_action);
				}
			}
		}
		insert_transitions(ret->actions, ret->homes);
		if(has_no_discontinuity(ret->actions, ret->homes)){
			return ret;
		} else {
			delete ret;
			return nullptr;
		}
	} catch (const std::exception& e) {
		delete ret;
		std::cout << e.what() << std::endl;
		return nullptr;
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

Action* parse_motion_action(ActionTypes type, double start, double end, unsigned long droneid, std::vector<std::string> split_line, double length_conversion, double angle_conversion) {
	Action* ret = nullptr;
	switch (type){
		case ActionTypes::Hover:
			ret = parse_hover_action(start, end, droneid, split_line, length_conversion);
			break;
		case ActionTypes::Line:
			ret = parse_line_action(start, end, droneid, split_line, length_conversion);
			break;
		case ActionTypes::Circle:
			ret = parse_circle_action(start, end, droneid, split_line, length_conversion, angle_conversion);
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
			ret = parse_simple_light_action(start, end, droneid, split_line);
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

Action* parse_hover_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion) {

	Point hover;
	hover.x() = std::atof(split_line[1].c_str());
	hover.y() = std::atof(split_line[3].c_str());
	hover.z() = std::atof(split_line[5].c_str());
	return new MotionAction(
					(unsigned) droneid,
					make_shared<LinearTrajectory>(hover*length_conversion, start, hover*length_conversion, end),
					ActionTypes::Hover);
}

Action* parse_line_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion) {
	Point start_point, end_point;
	start_point.x() = std::atof(split_line[1].c_str());
	start_point.y() = std::atof(split_line[3].c_str());
	start_point.z() = std::atof(split_line[5].c_str());
	end_point.x() = std::atof(split_line[7].c_str());
	end_point.y() = std::atof(split_line[9].c_str());
	end_point.z() = std::atof(split_line[11].c_str());
	return new MotionAction(
			(unsigned) droneid,
			make_shared<LinearTrajectory>(start_point*length_conversion, start, end_point*length_conversion, end),
			ActionTypes::Line);
}

Action* parse_circle_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line,double length_conversion,double angle_conversion) {
	Point origin;
	double theta1 = std::atof(split_line[3].c_str());
	double theta2 = std::atof(split_line[5].c_str());
	double radius = std::atof(split_line[1].c_str());
	origin.x() = std::atof(split_line[7].c_str());
	origin.y() = std::atof(split_line[9].c_str());
	origin.z() = std::atof(split_line[11].c_str());
	return new MotionAction(
			(unsigned) droneid,
			make_shared<CircleTrajectory>(origin * length_conversion, radius * length_conversion ,theta1 * angle_conversion, start, theta2 * angle_conversion, end),
			ActionTypes::Circle);
}

LightAction* parse_simple_light_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line){

}

LightAction* parse_strobe_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line){

}

bool has_no_discontinuity(std::vector<std::vector<Action*>>& actions, const std::vector<Point>& homes){
	std::vector<std::string> errors;
	auto floatComp = [](double a, double b) -> bool { return fabs(a-b) < 0.1; };
	auto pointComp = [](Point a, Point b) -> bool { return fabs((a-b).norm()) < 0.1; };
	for (unsigned j = 0; j < actions.size(); j++) {
		auto startPoint = homes[j];
		double startTime = 0.0;
		//Sort actions for each drone based on start time
		std::sort(actions[j].begin(), actions[j].end(),
				  [](Action *const &lhs, Action *const &rhs) { return lhs->GetStartTime() < rhs->GetStartTime(); });
		for (unsigned i = 0; i < actions[j].size(); i++) {
			Action *a = actions[j][i];
			double sTime = a->GetStartTime();
			double eTime = a->GetEndTime();
			//Check temporal continuity
			if (!is_light_action(a->GetActionType())) {
				if (!floatComp(sTime, startTime)) {
					errors.push_back(
							"Time Discontinuity for Drone: " + std::to_string(j) + " with start time: " +
							std::to_string(sTime) + ". Last command ended at : " + std::to_string(startTime));
				}
				startTime = eTime;
			}
			//Check spatial continuity
			if (!is_light_action(a->GetActionType())) {
				auto ma = dynamic_cast<MotionAction *>(a);
				auto actionStart = ma->GetStartPoint();
				if (!pointComp(actionStart, startPoint)) {
					errors.push_back(
							"Spatial Discontinuity for Drone: " + std::to_string(j) + ". Jumping from point: " +
							"[" + std::to_string(startPoint.x()) + " " + std::to_string(startPoint.y()) + " " +
							std::to_string(startPoint.z()) + "]" +
							" to point: " "[" + std::to_string(actionStart.x()) + " " +
							std::to_string(actionStart.y()) + " " + std::to_string(actionStart.z()) + "]" + "\n"
							+ "at start time: " + std::to_string(sTime)
					);
				}
				startPoint = ma->GetEndPoint();
			}
		}
	}
	for(const auto& s : errors){
		std::cout << s << std::endl;
	}
	return (errors.size() == 0);
}

void insert_transitions(std::vector<std::vector<Action*>>& actions, const std::vector<Point>& homes ){
	// Go back and review actions such that transitions can be created with polynomial trajectories
	for (unsigned i = 0; i < actions.size(); i++) {
		// Each entry in "actions" has a vector full of actions for that drone
		for (unsigned j = 0; j < actions[i].size(); j++) {
			if (!actions[i][j]->IsCalculated()) {
				double thisStart = actions[i][j]->GetStartTime();
				double thisEnd = actions[i][j]->GetEndTime();
				if (j == 0) {
					MotionAction *next = static_cast<MotionAction *>(actions[i][j + 1]);
					auto endState = next->GetPathState(next->GetStartTime());
					delete actions[i][j];
					actions[i][j] = new MotionAction(i,
													 PolynomialTrajectory::compute(
															 {homes[i]},
															 thisStart,
															 {endState.position, endState.velocity,
															  endState.acceleration},
															 thisEnd
													 ),
													 ActionTypes::Transition
					);
				} else if (j == (actions[i].size() - 1)) {
					//TODO: Handle the case of the last action being a transition, where our ending velocity and acceleration are 0 and destination is home.
				} else {
					// Calculate previous and next motion to generate what's needed for the transition
					// - can safely assume all are motions since made separate array for lights
					MotionAction* prev = dynamic_cast<MotionAction*>(actions[i][j - 1]);
					MotionAction* next = dynamic_cast<MotionAction*>(actions[i][j + 1]);
					// Get states from the previous and next state that were found
					auto startState = prev->GetPathState(prev->GetEndTime());
					auto endState = next->GetPathState(next->GetStartTime());
					// Cleanup object and replace with a new MotionAction
					delete actions[i][j];
					actions[i][j] = new MotionAction(
							i,
							PolynomialTrajectory::compute(
								{startState.position, startState.velocity, startState.acceleration},
								thisStart,
								{endState.position, endState.velocity, endState.acceleration},
								thisEnd),
							ActionTypes::Transition);
				}
			}
		}
	}
}

Point parse_point(std::string point) {
	boost::erase_all(point, "(");
	boost::erase_all(point, ")");
	boost::erase_all(point, "\"");
	boost::erase_all(point, " ");
	auto split_point = split(point, ';');
	Point ret;
	ret.x() = std::atof(split_point[0].c_str());
	ret.y() = std::atof(split_point[1].c_str());
	ret.z() = std::atof(split_point[2].c_str());
	return ret;
}

}
