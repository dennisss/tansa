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
	static std::vector<std::unique_ptr<Action>> Parse(const std::string &jocsPath);
private:
	/**
	 * Parse all actions in a given Jocs json object
	 * @param data Jocs json data
	 * @param actions [out] Will be filled with actions
	 */
	static void parseActions(const nlohmann::json &data, std::vector<std::unique_ptr<Action>>& actions);
	/**
	 * Parses a singular action array
	 * @private
	 * @param data Json data containing a reference to an action
	 * @param actions [out] Will be filled with actions
	 */
	static void parseAction(const nlohmann::json::reference data, std::vector<std::unique_ptr<Action>>& actions);
	/**
	 * Converts from string to ActionTypes enum
	 * @private
	 * @param data A string containing the ascii representation of an ActionType
	 * @return Action type as the enum
	 */
	static ActionTypes convertToActionType(const std::string& data);
};
}
#endif //TANSA_JOCSPARSER_H
