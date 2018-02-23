#ifndef FLIGHT_PLAN
#define FLIGHT_PLAN

#include"utillity.hpp"

enum direction
{
    cw,         // clockwise
    ccw,        // counter clockwise
    shortest    // take the shortest route
};


struct flightCommand
{
    direction rotationDirection;    // Direction to rotate
    int heading;                  // Ending heading, in degrees
    int rollTime;                 // Time to get to new heading, in milliseconds
};

/// if (validFlightPlan) Ensures:
///     - size() > 0
///     - 0 <= op[] < size() returns valid flightCommand
///     - 0 <= flightCommand.heading < 360
class flightplan
{
public:
    flightplan()
    {
        valid = false;
        commands = new flightCommand[0];
    }

    ~flightplan()
    {
        delete commands;
    }

    const flightCommand operator[](int i) const;

    int size() const
    {
        return numberOfFlightCommands;
    }

    bool validFlightPlan() const
    {
        return valid;
    }

    /// Parse a flight plan string format:
    /// '#3;+0901000;-0001000;~1802000;\0'
    void parseFlightPlan(char*);
    void beginRotation(unsigned long);
    int getTargetAngle(unsigned long);

private:
    bool valid;
    int numberOfFlightCommands;
    flightCommand* commands;

    int currentMove;
    unsigned long moveStartTime, moveEndTime;
};

#endif
