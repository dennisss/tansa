//
// Created by TessD on 10/14/2016.
//

#ifndef TANSA_ACTION_H
#define TANSA_ACTION_H

#include "trajectory.h"

typedef unsigned DroneId;
enum class LightId {TOP, BOT};

class Action {
public:
    Action(DroneId id): droneId(id) {}

    virtual double GetStartTime() = 0;
    virtual double GetEndTime() = 0;
    DroneId GetDrone() { return droneId; }

    bool IsCalculated() { return isCalculated; }

private:
    DroneId droneId;
    bool isCalculated;
};

class MotionAction : Action {
public:
    MotionAction(DroneId id, Trajectory t): droneId(id), path(t), isCalculated(true) {}

    Trajectory GetPath();
    virtual double GetStartTime() { return path.startTime(); }
    virtual double GetEndTime() { return path.endTime(); }

private:
    Trajectory path;
};

class LightAction : Action {
public:
    Light(double s, double d, DroneId id, double i, lightId l):
            startTime(s), duration(d), droneId(id), intensity(i),
            lightId(i), isCalculated(true) {}

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

#endif //TANSA_ACTION_H
