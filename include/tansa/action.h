//
// Created by TessD on 10/14/2016.
//

#ifndef TANSA_ACTION_H
#define TANSA_ACTION_H

#include "trajectory.h"
#include <memory>

namespace tansa {
	typedef unsigned DroneId;
	enum class LightId {TOP, BOT};
	class Action {
	public:
		Action(DroneId id) : droneId(id) {}

		virtual double GetStartTime() = 0;

		virtual double GetEndTime() = 0;

		DroneId GetDrone() { return droneId; }

		bool IsCalculated() { return isCalculated; }

	protected:
		DroneId droneId;
		bool isCalculated;
	};

	class MotionAction : Action {
	public:
		MotionAction(DroneId id, std::unique_ptr<Trajectory> t) : Action(id), path(std::move(t)) { isCalculated = true;}

		std::unique_ptr<Trajectory> GetPath();

		virtual double GetStartTime() { return path->startTime(); }

		virtual double GetEndTime() { return path->endTime(); }

	private:
		std::unique_ptr<Trajectory> path;
	};

	class LightAction : Action {
	public:
		LightAction(double s, double e, DroneId id, double i, LightId l) :  Action(id),
				startTime(s), endTime(e), intensity(i), lightId(l) {isCalculated = true;}

		virtual double GetStartTime() { return startTime; }

		virtual double GetEndTime() { return endTime; }

		double GetIntensity() { return intensity; }

		LightId GetLightId() { return lightId; }

	private:
		double startTime;
		double endTime;
		double intensity;
		LightId lightId;
	};
}

#endif //TANSA_ACTION_H
