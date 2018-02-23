#include <c++/6/iostream>
#include "flightplan.hpp"

/// Utility Functions

bool isDigit(char c)
{
    return '0' <= c && c <= '9';
}

bool areDigits(char* c, int n)
{
    for (int i = 0; i < n; ++i)
    {
        if (!isDigit(c[i]))
        {
            return false;
        }
    }
    return true;
}

int appendCharDigit(int number, char c)
{
    if (isDigit(c))
    {
        number *= 10;
        number += c - '0';
    }
    return number;
}

int getNumberFromChars(char* c, int n)
{
    int result = 0;
    for (int i = 0; i < n; ++i)
    {
        result = appendCharDigit(result, c[i]);
    }
    return result;
}


///flightplan Implementation

const

flightCommand flightplan::operator[](int i) const
{
    if (0 <= i && i < size())
    {
        return commands[i];
    }
    return flightCommand();
}

//void flightplan::hash()
//{

//}

void flightplan::parseFlightPlan(char* plan)
{
    /// Returning before the last line of the method will
    /// leave the validFlightPlan() == false.

    int pos = 0;

    // Parse '#number;'
    if (plan[pos++] != '#')
    {
        // missing #, invalid plan
        return;
    }

    numberOfFlightCommands = 0;
    // While the char is a number;
    while (isDigit(plan[pos]))
    {
        // If this is the end of the string.
        if (plan[pos] == 0)
        {
            // short circuit, invalid plan
            return;
        }
        numberOfFlightCommands = appendCharDigit(numberOfFlightCommands, plan[pos]);
        ++pos;
    }

    // Move off of ';'
    pos++;

    // Get rid of the old flight data.
    delete commands;

    // Allocate space for new flight commands.
    commands = new flightCommand[numberOfFlightCommands];

    // Read each flight command.
    for (int i = 0; i < numberOfFlightCommands; ++i)
    {
        // Parse rotation direction.
        switch (plan[pos])
        {
            case '+':
                commands[i].rotationDirection = cw;
                break;
            case '-':
                commands[i].rotationDirection = ccw;
                break;
            case '~':
                commands[i].rotationDirection = shortest;
                break;
            default:
                // Invalid rotation direction.
                return;
        }
        ++pos;

        // Parse end orientation.
        // Verify next 3 chars are digits.
        if (areDigits(&plan[pos], 3))
        {
            commands[i].heading = getNumberFromChars(&plan[pos], 3) % 360;
            pos += 3;
        }
        else
        {
            // Invalid degree angle.
            return;
        }

        // Parse time.
        if (areDigits(&plan[pos], 4))
        {
            commands[i].rollTime = getNumberFromChars(&plan[pos], 4);
            pos += 4;
        }
        else
        {
            // Invalid milli second
            return;
        }


        // Check semi-colon.
        if (plan[pos++] != ';')
        {
            return;
        }
    }

    // If it gets here, all formatting is correct.
    valid = true;
}

void flightplan::beginRotation(unsigned long time)
{
    // Move to the first command.
    currentMove = 0;
    // Save current time in millis.
    moveStartTime = time;
    // Calculate and save end of first move;
    moveEndTime = time + commands[currentMove].rollTime;
}

int flightplan::getTargetAngle(unsigned long time)
{
    if (currentMove < 0
        || currentMove >= numberOfFlightCommands)
    {
        // return invalid angle.
        return -1;
    }

    // If finished the last move, progress to next stage.
    if (time > moveEndTime)
    {
        // Move to next step of flight plan.
        currentMove++;
        // Set the end time of the last move to the start time of this move.
        moveStartTime = moveEndTime;
        // Calculate and save end of next move.
        moveEndTime = moveStartTime + commands[currentMove].rollTime;
    }

    // Calculate the target angle.
    flightCommand& move = commands[currentMove];
    flightCommand lastMove;
    if (currentMove == 0)
    {
        lastMove.heading = 0;
    }
    else
    {
        lastMove = commands[currentMove - 1];
    }

    if (move.rotationDirection == shortest)
    {
        // If we don't care about the roll direction
        // get there as quickly as possible.
        return move.heading;
    }

    int relativeRotation = move.heading - lastMove.heading;
    unsigned long turnTime = time - moveStartTime;
    int progressAngle = relativeRotation * ((double) turnTime / (double) move.rollTime);
    int targetAngle = lastMove.heading + progressAngle;

    if (targetAngle < 0)
    {
        targetAngle += 360;
    }
    targetAngle %= 360;

    std::cerr << targetAngle << std::endl;
    return targetAngle;
}
