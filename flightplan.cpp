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

// Utility Functions for Config Parsing

bool isFpVital(char e){
    if (isDigit(e)) return true;
    if (e == '.') return true;
    if(e == '+') return true;
    if (e == '-') return true;
    if (e == '~') return true;
    if (e == '#') return true;
    return false;
}

int getCaSize(char* str){
    int i = 0;
    while(str[i] != '\0'){ ++i; }
    ++i;
    return i;
}

char* caAppend(char* in, char e){
    /*Appends to a char* allocated on the heap*/
    if (in == nullptr){
        in = new char[2];
        in[0] = e;
        in[1] = '\0';
        return in;
    }
    else{
        int size = getCaSize(in);
        char* temp = new char[size+2];
        int i = 0;
        for(; i < size-1; ++i){ temp[i] = in[i]; }
        delete[] in;
        in = temp;
        in[i] = e;
        in[i+1] = '\0';
        return in;
    }
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
        if (areDigits(&plan[pos],4))
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
