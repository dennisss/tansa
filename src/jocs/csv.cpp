//
// Created by kyle on 2/28/17.
//

#include "tansa/csv.h"

#include <algorithm>


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

// Remove all instances of c from the string
// TODO: Make this more efficient
void erase_all(std::string &s, char c) {
	for(int i = 0; i < s.size(); i++) {
		if(s[i] != c) {
			s.erase(i, 1);
			i--;
		}
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
	else if (data == "ellipse")
		return ActionTypes::Ellipse;
	else if (data == "spiral")
		return ActionTypes::Spiral;
	else if (data == "helix")
		return ActionTypes::Helix;
	else if (data == "arc")
		return ActionTypes::Arc;
	else if (data == "trajectory")
		return ActionTypes::TransformedTraj;
	else if (data == "fade")
		return ActionTypes::Fade;
	else if(data == "strobe_dynamic")
		return ActionTypes::DynamicStrobe;
	else{
		std::cout << "Unknown action type: " << data << std::endl;
		return ActionTypes::None;
	}
}

bool is_light_action(ActionTypes type){
	return type == ActionTypes::Strobe || type == ActionTypes::Light || type == ActionTypes::Fade || type == ActionTypes::DynamicStrobe;
}

// Credit to http://stackoverflow.com/questions/6089231/getting-std-ifstream-to-handle-lf-cr-and-crlf
std::istream& safeGetline(std::istream& is, std::string& t) {
	t.clear();

	// The characters in the stream are read one-by-one using a std::streambuf.
	// That is faster than reading them one-by-one using the std::istream.
	// Code that uses streambuf this way must be guarded by a sentry object.
	// The sentry object performs various tasks,
	// such as thread synchronization and updating the stream state.

	std::istream::sentry se(is, true);
	std::streambuf* sb = is.rdbuf();

	for(;;) {
		int c = sb->sbumpc();
		switch (c) {
			case '\n':
				return is;
			case '\r':
				if(sb->sgetc() == '\n')
					sb->sbumpc();
				return is;
			case EOF:
				// Also handle the case when the last line has no line ending
				if(t.empty())
					is.setstate(std::ios::eofbit);
				return is;
			default:
				t += (char)c;
		}
	}
}

bool read_csv_line(ifstream &file, std::vector<string> &split_line, unsigned skip_cols = 0){
	std::string line;
	if(safeGetline(file, line).eof()) {
		return false;
	}

	auto ret = split(line, ',');
	unsigned n = ret.size() - skip_cols;
	split_line.resize(n);
	for(unsigned i = 0; i < n; i++) {
		split_line[i] = ret[i + skip_cols];
	}

	return true;
}

Choreography* parse_csv(const char* filepath, double scale){
	Choreography* ret = nullptr;
	int currentLine = 1; // Lines indexed at 1
	try {
		ifstream csv(filepath);
		ret = new Choreography();
		auto drone_map = std::map<std::string, unsigned long>();
		std::vector<std::string> split_line;
		read_csv_line(csv, split_line); //Contains dronekey and meters/feet
		unsigned skip_cols = 0; // Number of columns to skip per line
		for(unsigned i = 0; i < split_line.size(); i++) {
			if(split_line[i] == "Breakpoint") { // Skip all extra columns before the breakpoints
				skip_cols = i;
				break;
			}
		}
		// Remove skipped columns for the first line
		for(unsigned i = 0; i < skip_cols; i++)
			split_line.erase(split_line.begin());

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
		size_t start_index = csv_positions::DroneKeyPos;
		unsigned long num_drones = 0;
		for (size_t i = start_index; i < split_line.size(); i++) {
			if (split_line[i].empty())
				break;
			drone_map[split_line[i]] = num_drones;
			num_drones++;
		}
		ret->actions.resize(num_drones);
		ret->lightActions.resize(num_drones);
		for(int i = 0; i < num_drones; i++) {
			ret->lightActions[i].resize(LightController::NUM_LIGHTS);
		}
		currentLine++;

		std::vector<std::string> home_split;
		read_csv_line(csv, home_split, skip_cols);
		for (size_t i = csv_positions::ParamStartPos; i < home_split.size(); i++) {
			ret->homes.push_back(parse_point(home_split[i])*conversion_factor);
			if (ret->homes.size() == num_drones)
				break;
		}
		currentLine++;

		while (read_csv_line(csv, split_line, skip_cols)) { //Iterate line by line

			for(int i = 0; i < split_line.size(); i++)
				erase_all(split_line[i], '\r');
			//Parse breakpoints
			if (!split_line[csv_positions::BreakpointPos].empty()) {
				ret->breakpoints.push_back(
						Breakpoint(split_line[csv_positions::BreakpointPos],
								   break_num++,
								   parse_time(split_line[csv_positions::StartTimePos])));
			}

			// Skipping any lines without time information (probably empty)
			if(split_line.size() < csv_positions::DronesPos) {
				continue;
			}
			bool emptyHead = true;
			for(int i = 0; i <= csv_positions::DronesPos; i++) {
				if(!split_line[i].empty()) {
					emptyHead = false;
					break;
				}
			}
			if(emptyHead) {
				continue;
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
					auto light_index = light_action->GetLightIndex();
					ret->lightActions[drones[j]][light_index].push_back(light_action);
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
					if(motion_action != nullptr)
						motion_action->line = currentLine;
					ret->actions[drones[j]].push_back(motion_action);
				}
			}

			currentLine++;
		}
		insert_transitions(ret->actions, ret->homes);
		fill_light_gaps(ret->lightActions,(*(ret->actions[0].end()-1))->GetEndTime());

		/*
		for (const auto& a : ret->lightActions[0][0]){

			printf("%f %f \n", a->GetStartTime(), a->GetEndTime() );
		}
		*/
		return ret;

	} catch (const std::exception& e) {
		delete ret;
		std::cout << "Line " << currentLine << ": " << e.what() << std::endl;
		return nullptr;
	}
}

double parse_time(std::string& time){
	return std::atof(time.c_str());
}

std::vector<unsigned long> parse_drones(std::string drone_field, const std::map<std::string, unsigned long>& drone_map) {
	erase_all(drone_field, '(');
	erase_all(drone_field, ')');
	erase_all(drone_field, '"');
	erase_all(drone_field, ' ');
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
		case ActionTypes::Arc:
			ret = parse_arc_action(start, end, droneid, split_line, length_conversion);
			break;
		case ActionTypes::Ellipse:
			ret = parse_ellipse_action(start, end, droneid, split_line, length_conversion, angle_conversion);
			break;
		case ActionTypes::Spiral:
			ret = parse_spiral_action(start, end, droneid, split_line, length_conversion, angle_conversion);
			break;
		case ActionTypes::TransformedTraj:
			ret = parse_trajectory_action(start, end, droneid, split_line, length_conversion, angle_conversion);
			break;
		case ActionTypes::Helix:
			ret = parse_helix_action(start, end, droneid, split_line, length_conversion, angle_conversion);
			break;
		case ActionTypes::None:
			ret = nullptr;
		default:
			ret = nullptr;
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
		case ActionTypes::DynamicStrobe:
			ret = parse_dynamic_strobe_action(start, end, droneid, split_line);
			break;
		case ActionTypes::Fade:
			ret = parse_fade_action(start, end, droneid, split_line);
			break;
		default:
			ret = nullptr;
			break;
	}
	return ret;
}

Action* parse_hover_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion) {
	enum indices : unsigned {
		start_x_loc 	= 1,
		start_y_loc 	= 2,
		start_z_loc 	= 3,
	};

	if(split_line.size() < start_z_loc + 1) {
		throw std::runtime_error("Not enough fields in hover action");
	}

	Point hover;
	hover.x() 			= std::atof(split_line[start_x_loc].c_str());
	hover.y() 			= std::atof(split_line[start_y_loc].c_str());
	hover.z() 			= std::atof(split_line[start_z_loc].c_str());
	hover				= hover * length_conversion;
	return new MotionAction(
					(unsigned) droneid,
					make_shared<LinearTrajectory>(hover, start, hover, end),
					ActionTypes::Hover);
}

Action* parse_line_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion) {
	enum indices : unsigned {
		start_x_loc 	= 1,
		start_y_loc 	= 2,
		start_z_loc 	= 3,
		end_x_loc 		= 5,
		end_y_loc 		= 6,
		end_z_loc 		= 7
	};

	if(split_line.size() < end_z_loc + 1) {
		throw std::runtime_error("Not enough fields in line action");
	}


	Point start_point, end_point;
	start_point.x() 	= std::atof(split_line[start_x_loc].c_str());
	start_point.y() 	= std::atof(split_line[start_y_loc].c_str());
	start_point.z() 	= std::atof(split_line[start_z_loc].c_str());
	end_point.x() 		= std::atof(split_line[end_x_loc].c_str());
	end_point.y() 		= std::atof(split_line[end_y_loc].c_str());
	end_point.z() 		= std::atof(split_line[end_z_loc].c_str());
	start_point 		= start_point * length_conversion;
	end_point 			= end_point * length_conversion;
	return new MotionAction(
			(unsigned) droneid,
			make_shared<LinearTrajectory>(start_point, start, end_point, end),
			ActionTypes::Line);
}

Action* parse_circle_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line,double length_conversion,double angle_conversion) {
	enum indices : unsigned {
		radius_loc 		= 1,
		theta1_loc 		= 3,
		theta2_loc 		= 5,
		x_loc 			= 7,
		y_loc 			= 8,
		z_loc 			= 9
	};

	if(split_line.size() < z_loc + 1) {
		throw std::runtime_error("Not enough fields in circle action");
	}

	Point origin;
	double radius 		= std::atof(split_line[radius_loc].c_str()) * length_conversion;
	double theta1 		= std::atof(split_line[theta1_loc].c_str()) * angle_conversion;
	double theta2 		= std::atof(split_line[theta2_loc].c_str()) * angle_conversion;
	origin.x() 			= std::atof(split_line[x_loc].c_str());
	origin.y() 			= std::atof(split_line[y_loc].c_str());
	origin.z() 			= std::atof(split_line[z_loc].c_str());
	origin 				= origin * length_conversion;
	return new MotionAction(
			(unsigned) droneid,
			make_shared<CircleTrajectory>(origin, radius,theta1, start, theta2, end),
			ActionTypes::Circle);
}

Action* parse_ellipse_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion, double angle_conversion) {
	enum indices : unsigned {
		radius_x_loc 	= 1,
		radius_y_loc 	= 3,
		theta1_loc 	 	= 5,
		theta2_loc	 	= 7,
		x_loc 		 	= 9,
		y_loc			= 10,
		z_loc 			= 11
	};
	Point origin;
	double radius_x 	= std::atof(split_line[radius_x_loc].c_str()) * length_conversion;
	double radius_y 	= std::atof(split_line[radius_y_loc].c_str()) * length_conversion;
	double theta1 		= std::atof(split_line[theta1_loc].c_str()) * angle_conversion;
	double theta2 		= std::atof(split_line[theta2_loc].c_str()) * angle_conversion;
	origin.x() 			= std::atof(split_line[x_loc].c_str());
	origin.y() 			= std::atof(split_line[y_loc].c_str());
	origin.z() 			= std::atof(split_line[z_loc].c_str());
	origin 				= origin * length_conversion;
	return new MotionAction(
			(unsigned) droneid,
			make_shared<EllipseTrajectory>(origin, radius_x, radius_y, theta1, start, theta2, end),
			ActionTypes::Ellipse);
}

Action* parse_helix_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion, double angle_conversion) {
	enum indices : unsigned {
		radius_loc 	= 1,
		theta1_loc 	 	= 3,
		theta2_loc	 	= 5,
		x_loc 		 	= 7,
		y_loc			= 8,
		z_loc 			= 9,
		x2_loc 		 	= 11,
		y2_loc			= 12,
		z2_loc 			= 13,
	};

	if(split_line.size() < z2_loc + 1) {
		throw std::runtime_error("Not enough fields in hover action");
	}

	Point origin, origin2;
	double radius 	= std::atof(split_line[radius_loc].c_str()) * length_conversion;
	double theta1 		= std::atof(split_line[theta1_loc].c_str()) * angle_conversion;
	double theta2 		= std::atof(split_line[theta2_loc].c_str()) * angle_conversion;
	origin.x() 			= std::atof(split_line[x_loc].c_str());
	origin.y() 			= std::atof(split_line[y_loc].c_str());
	origin.z() 			= std::atof(split_line[z_loc].c_str());
	origin 				= origin * length_conversion;

	origin2.x() 			= std::atof(split_line[x2_loc].c_str());
	origin2.y() 			= std::atof(split_line[y2_loc].c_str());
	origin2.z() 			= std::atof(split_line[z2_loc].c_str());
	origin2 				= origin2 * length_conversion;
	return new MotionAction(
			(unsigned) droneid,
			HelixHelper(radius, origin, theta1, start, origin2, theta2, end),
			ActionTypes::Helix);
}

Action* parse_spiral_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion, double angle_conversion){
	enum indices : unsigned {
		radius_loc 		= 1,
		x_loc 		 	= 3,
		y_loc			= 4,
		z_loc 			= 5,
		theta1_loc 	 	= 7,
		theta2_loc	 	= 9
	};

	if(split_line.size() < theta2_loc + 1) {
		throw std::runtime_error("Not enough fields in hover action");
	}

	Point origin;
	double radius	 	= std::atof(split_line[radius_loc].c_str()) * length_conversion;
	double theta1 		= std::atof(split_line[theta1_loc].c_str()) * angle_conversion;
	double theta2 		= std::atof(split_line[theta2_loc].c_str()) * angle_conversion;
	origin.x() 			= std::atof(split_line[x_loc].c_str());
	origin.y() 			= std::atof(split_line[y_loc].c_str());
	origin.z() 			= std::atof(split_line[z_loc].c_str());
	origin 				= origin * length_conversion;
	return new MotionAction(
			(unsigned) droneid,
			make_shared<SpiralTrajectory>(origin, radius, theta1, start, theta2, end),
			ActionTypes::Spiral);
}

Action* parse_arc_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion){
	constexpr unsigned NUM_PER_SEGMENT = 6;
	std::vector<ConstrainedPoint> points;
	std::vector<double> times;
	std::vector<string> line = split_line;
	//Remove all empty cells
	line.erase(std::remove_if(line.begin(), line.end(), [](const std::string& i) { return i.empty() || i == "\r";}), line.end());
	assert((line.size() % NUM_PER_SEGMENT) == 0);

	unsigned to_process = (unsigned)line.size() / NUM_PER_SEGMENT;
	unsigned processed 	= 0;
	auto process 		= [&](const std::vector<std::string> &sub_line) {
		enum indices : unsigned {
			time_loc 	= 1,
			x_loc 		= 3,
			y_loc		= 4,
			z_loc 		= 5
		};

		Point p;
		unsigned time 	= (unsigned)std::atof(sub_line[time_loc].c_str()); //TODO: warn about non integer times
		p.x() 			= std::atof(sub_line[x_loc].c_str());
		p.y()			= std::atof(sub_line[y_loc].c_str());
		p.z()			= std::atof(sub_line[z_loc].c_str());
		p 				= p * length_conversion;
		times.push_back(time);
		if(to_process == 1 || processed == 0) { //If it is the first or last point it must be stationary
			points.push_back(ConstrainedPoint::Stationary(p));
		} else {
			points.push_back({p});
		}
	};
	while (to_process > 0) {
		process(std::vector<std::string>(line.begin() + processed, line.begin() + processed + NUM_PER_SEGMENT));
		processed += NUM_PER_SEGMENT;
		to_process--;
	}
	Trajectory::Ptr out;
	if(compute_minsnap_mellinger11(points, times, {}, &out, NULL)) {
		return new MotionAction(
				(unsigned) droneid,
				out,
				ActionTypes::Arc
		);
	} else {
		//TODO: Return null here and handle above instead of just throwing an exception.
		throw std::runtime_error("Failed to compute Arc trajectory for drone id"
								 + std::to_string(droneid)
								 + "at start time: "
								 + std::to_string(start));
	}
}

Action* parse_trajectory_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line, double length_conversion, double angle_conversion){
	enum indices : unsigned {
		angle_x_loc 	= 1,
		angle_y_loc 	= 3,
		angle_z_loc 	= 5,
		offset_x_loc 	= 7,
		offset_y_loc 	= 9,
		offset_z_loc 	= 11,
		traj_loc		= 12,
	};

	if(split_line.size() < traj_loc + 1) {
		throw std::runtime_error("Not enough fields in transform action");
	}

	Trajectory::Ptr ptr = dynamic_cast<MotionAction*>(parse_motion_action(
			parse_action_type(split_line[traj_loc]),
			start,
			end,
			droneid,
			std::vector<std::string>(split_line.begin()+ traj_loc + 1, split_line.end()),
			length_conversion,
			angle_conversion))->GetPath();

	double angle_x 			= std::atof(split_line[angle_x_loc].c_str()) * angle_conversion;
	double angle_y 			= std::atof(split_line[angle_y_loc].c_str()) * angle_conversion;
	double angle_z 			= std::atof(split_line[angle_z_loc].c_str()) * angle_conversion;
	double trans_x 			= std::atof(split_line[offset_x_loc].c_str()) * length_conversion;
	double trans_y 			= std::atof(split_line[offset_y_loc].c_str()) * length_conversion;
	double trans_z 			= std::atof(split_line[offset_z_loc].c_str()) * length_conversion;
	Vector3d translation 	= {trans_x, trans_y, trans_z};
	AngleAxisd x_rotation 	= AngleAxisd(angle_x, Vector3d::UnitX());
	AngleAxisd y_rotation 	= AngleAxisd(angle_y, Vector3d::UnitY());
	AngleAxisd z_rotation 	= AngleAxisd(angle_z, Vector3d::UnitZ());
	Quaterniond quat 		= x_rotation * y_rotation * z_rotation;

	return new MotionAction(
			(unsigned) droneid,
			make_shared<TransformedTrajectory>( ptr, quat.matrix(), translation, start, end),
			ActionTypes::TransformedTraj
	);
}

LightAction* parse_light_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line){
	enum indices : unsigned {
		index_loc = 1,
		R_loc 	= 3,
		G_loc 	= 4,
		B_loc 	= 5,
		i_loc 	= 7
	};

	if(split_line.size() < i_loc + 1) {
		throw std::runtime_error("Not enough fields in the light action");
	}

	auto index = parse_light_index(split_line[index_loc]);
	int r = std::atoi(split_line[R_loc].c_str());
	int g = std::atoi(split_line[G_loc].c_str());
	int b = std::atoi(split_line[B_loc].c_str());
	double i = std::atof(split_line[i_loc].c_str());

	Color col = Color::from_8bit_colors(r,g,b);

	LightTrajectory::Ptr p = make_shared<LightTrajectory>(i, col, start, i, col, end);

	return new LightAction(droneid, p, index);
}

LightAction* parse_fade_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line){
	enum indices : unsigned {
		index_loc = 1,
		sR_loc 	= 3,
		sG_loc 	= 4,
		sB_loc 	= 5,
		si_loc 	= 7,
		eR_loc 	= 9,
		eG_loc 	= 10,
		eB_loc 	= 11,
		ei_loc 	= 13
	};

	if(split_line.size() < ei_loc + 1) {
		throw std::runtime_error("Not enough fields in the fade action");
	}
	auto index = parse_light_index(split_line[index_loc]);
	int sr = std::atoi(split_line[sR_loc].c_str());
	int sg = std::atoi(split_line[sG_loc].c_str());
	int sb = std::atoi(split_line[sB_loc].c_str());
	double si = std::atof(split_line[si_loc].c_str());

	int er = std::atoi(split_line[eR_loc].c_str());
	int eg = std::atoi(split_line[eG_loc].c_str());
	int eb = std::atoi(split_line[eB_loc].c_str());
	double ei = std::atof(split_line[ei_loc].c_str());

	Color sCol = Color::from_8bit_colors(sr,sg,sb);
	Color eCol = Color::from_8bit_colors(er,eg,eb);

	LightTrajectory::Ptr p = make_shared<LightTrajectory>(si, sCol, start, ei, eCol, end);

	return new LightAction(droneid, p, index);
}

LightAction* parse_strobe_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line){
	enum indices : unsigned {
		index_loc = 1,
		sR_loc 	= 3,
		sG_loc 	= 4,
		sB_loc 	= 5,
		si_loc 	= 7,
		eR_loc 	= 9,
		eG_loc 	= 10,
		eB_loc 	= 11,
		ei_loc 	= 13,
		cps_loc = 15,
	};

	if(split_line.size() < cps_loc + 1) {
		throw std::runtime_error("Not enough fields in the fade action");
	}
	auto index = parse_light_index(split_line[index_loc]);
	int sr = std::atoi(split_line[sR_loc].c_str());
	int sg = std::atoi(split_line[sG_loc].c_str());
	int sb = std::atoi(split_line[sB_loc].c_str());
	double si = std::atof(split_line[si_loc].c_str());

	int er = std::atoi(split_line[eR_loc].c_str());
	int eg = std::atoi(split_line[eG_loc].c_str());
	int eb = std::atoi(split_line[eB_loc].c_str());
	double ei = std::atof(split_line[ei_loc].c_str());
	double cps = std::atof(split_line[cps_loc].c_str());

	Color sCol = Color::from_8bit_colors(sr,sg,sb);
	Color eCol = Color::from_8bit_colors(er,eg,eb);

	LightTrajectory::Ptr p = make_shared<StrobeTrajectory>(si, sCol, start, ei, eCol, end, cps, cps);

	return new LightAction(droneid, p, index);
}

LightAction* parse_dynamic_strobe_action(double start, double end, unsigned long droneid, const std::vector<std::string>& split_line){
	enum indices : unsigned {
		index_loc = 1,
		sR_loc 	= 3,
		sG_loc 	= 4,
		sB_loc 	= 5,
		si_loc 	= 7,
		cpss_loc = 9,
		eR_loc 	= 11,
		eG_loc 	= 12,
		eB_loc 	= 13,
		ei_loc 	= 15,
		cpse_loc = 17,
	};

	if(split_line.size() < cpse_loc + 1) {
		throw std::runtime_error("Not enough fields in the fade action");
	}
	auto index = parse_light_index(split_line[index_loc]);
	int sr = std::atoi(split_line[sR_loc].c_str());
	int sg = std::atoi(split_line[sG_loc].c_str());
	int sb = std::atoi(split_line[sB_loc].c_str());
	double si = std::atof(split_line[si_loc].c_str());

	int er = std::atoi(split_line[eR_loc].c_str());
	int eg = std::atoi(split_line[eG_loc].c_str());
	int eb = std::atoi(split_line[eB_loc].c_str());
	double ei = std::atof(split_line[ei_loc].c_str());
	double cpss = std::atof(split_line[cpss_loc].c_str());
	double cpse = std::atof(split_line[cpse_loc].c_str());

	Color sCol = Color::from_8bit_colors(sr,sg,sb);
	Color eCol = Color::from_8bit_colors(er,eg,eb);

	LightTrajectory::Ptr p = make_shared<StrobeTrajectory>(si, sCol, start, ei, eCol, end, cpss, cpse);

	return new LightAction(droneid, p, index);
}

void insert_transitions(std::vector<std::vector<Action*>>& actions, const std::vector<Point>& homes ){
	// Go back and review actions such that transitions can be created with polynomial trajectories
	for (unsigned i = 0; i < actions.size(); i++) {
		// Each entry in "actions" has a vector full of actions for that drone
		for (unsigned j = 0; j < actions[i].size(); j++) {
			if (!actions[i][j]->IsCalculated()) {
				int line = actions[i][j]->line;

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

				actions[i][j]->line = line;

			}
		}
	}
}

Point parse_point(std::string point) {
	std::string point_clean = point;
	erase_all(point_clean, '(');
	erase_all(point_clean, ')');
	erase_all(point_clean, '"');
	erase_all(point_clean, ' ');
	Point ret;
	auto split_point 	= split(point_clean, ';');

	if(split_point.size() != 3) {
		throw std::runtime_error("Invalid Point: " + point);
	}

	ret.x() 			= std::atof(split_point[0].c_str());
	ret.y() 			= std::atof(split_point[1].c_str());
	ret.z() 			= std::atof(split_point[2].c_str());
	return ret;
}

LightController::LightIndices parse_light_index(const std::string& in){

	std::string str = in;
	std::transform(str.begin(), str.end(), str.begin(), ::tolower);

	if(str == "left")
		return LightController::LightIndices::LEFT;
	if(str == "right")
		return LightController::LightIndices::RIGHT;
	if(str == "internal")
		return LightController::LightIndices::INTERNAL;
	throw std::runtime_error("Failed to parse light index: " + in);
}

void fill_light_gaps(std::vector<std::vector<std::vector<LightAction*>>>& actions, double end_time){
	//For each drone
	for (auto i = 0; i < actions.size(); i++){
		//For each light on each drone
		for(auto j = 0; j < actions[i].size(); j++){
			if(actions[i][j].size() == 0){
				//TODO Fix drone ID I think i might not be correct. But it might not matter for light actions at this point.
				// Maybe matters in future though?
				actions[i][j].push_back(new LightAction(i, make_shared<EmptyLightTrajectory>(0.0, end_time),(LightController::LightIndices)j));
			} else if(fabs((*(actions[i][j].end()-1))->GetEndTime() - end_time) > 0.01) {
				actions[i][j].push_back(new LightAction(i, make_shared<EmptyLightTrajectory>((*(actions[i][j].end() - 1))->GetEndTime(), end_time), (LightController::LightIndices)j));
			} else {
				double et = actions[i][j][0]->GetEndTime();
				//For each action for this light for this drone
				for (auto k = 1; k < actions[i][j].size(); k++) {
					double st = actions[i][j][k]->GetStartTime();
					double temp = fabs(st - et);
					if ( temp > 0.01) {
						auto it = actions[i][j].begin() + k;
 						unsigned droneid = actions[i][j][k]->GetDrone();
						double new_et = 0;
						if (k < actions[i][j].size()) {
							new_et = actions[i][j][k]->GetStartTime();
						} else {
							new_et = end_time;
						}
						actions[i][j].insert(it, new LightAction(droneid, make_shared<EmptyLightTrajectory>(et, new_et),
																 (LightController::LightIndices) j));
						et = new_et;
					} else {
						et = actions[i][j][k]->GetEndTime();
					}
				}
			}
		}
	}
}
}
