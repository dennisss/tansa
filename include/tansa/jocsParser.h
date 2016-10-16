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
 * Type representing the different actions in jocs
 * @enum
 */
enum ActionTypes : unsigned{
	Transition = 0,
	Line = 1,
	Circle = 2,
};

/**
 * Class representing the
 * @static
 */
class Jocs {
public:
	static const std::string HOME_KEY;
	static const std::string DRONE_KEY;
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
	/**
	 * Parses a given Jocs file
	 * @public
	 * @param jocsPath String containing path from working directory to jocsFile
	 * @return A vector containing unique_ptrs to Actions
	 */
	static std::vector<Action*> Parse(const std::string &jocsPath);
private:
	/**
	 * Parse all actions in a given Jocs json object
	 * @param data [in] Jocs json data
	 * @param actions [out] Will be filled with actions
	 */
	static void parseActions(const nlohmann::json &data, std::vector<Action*>& actions);
	/**
	 * Parses a singular action array
	 * @private
	 * @param data Json data containing a reference to an action
	 * @param actions [out] Will be filled with actions
	 */
	static void parseAction(const nlohmann::json::reference data, std::vector<Action*>& actions);
	/**
	 * Converts from string to ActionTypes enum
	 * @private
	 * @param data A string containing the ascii representation of an ActionType
	 * @return Action type as the enum
	 */
	static ActionTypes convertToActionType(const std::string& data);
	/**
	 * Cycles 2D vector to find the action directly previous to a current action for a specific drone
	 * @private
	 * @param id The id of the drone we want to search for actions for
	 * @param actions The 2d array full of actions organized by their start time
	 * @param currentLocation The start time we want to find the previous action before
	 * @return Action that ends at the time that a drone's current actions starts
	 */
	static MotionAction* Jocs::FindPreviousAction(DroneId id, std::vector< vector<Action*> >& actions, int currentLocation);
	/**
	 * Cycles 2D vector to find the action directly after a current action for a specific drone
	 * @private
	 * @param id The id of the drone we want to search for actions for
	 * @param actions The 2d array full of actions organized by their start time
	 * @param currentLocation The start time we want to find the next action after
	 * @return Action that starts at the time that a drone's current actions ends
	 */
	static MotionAction* Jocs::FindNextAction(DroneId id, std::vector< vector<Action*> >& actions, int currentLocation);
	/**
	 * Cycles Action vector to find the Action pertaining to a specific drone id
	 * @private
	 * @param id The id of the drone we want to search for actions for
	 * @param curActions The vector full of actions at the same start time
	 * @param actionFound The Action within curActions which applies to the intended drone id
	 * @return Action that starts at the time that a drone's current actions ends
	 */
	static bool Jocs::FindMotionForDrone(DroneId id, vector<Action*> curActions, MotionAction& actionFound);
};
}
#endif //TANSA_JOCSPARSER_H
