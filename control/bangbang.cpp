//
// Created by alex on 4/29/18.
//
#include "bangbang.hpp"

const int CENTER = 180.0;
#define tolerance 7

/*
 *  Put the current angle in the frame of 180. Works
 *  the same way a combination lock would.
 */
int frame180(const int targetAngle, const int currentAngle)
{
    int offset = CENTER - targetAngle;
    int shiftedAngle = currentAngle + offset;
    if (shiftedAngle < 0) // negative numbers must wrap around
    {
        shiftedAngle = 360 + shiftedAngle;
    }
    else if (360 <= shiftedAngle) // Cover 0/360 and going above 360.
    {
        shiftedAngle %= 360;
    }
    return shiftedAngle;
}

/*
 *  Give a direction for the fins to turn.
 *  +, -, or 0
 */
int finAngleBased180(float currentAngle)
{
    int offAngle = currentAngle - CENTER;

    if (offAngle > tolerance)
    {
        return 1;
    }
    else if (offAngle < -tolerance)
    {
        // Need to turn left.
        return -1;
    }
    else
    {
        return 0;
    }
}

int getFinAngle(int targetAngle, float currentAngle)
{
    currentAngle = frame180(targetAngle, currentAngle);
    return finAngleBased180(currentAngle);
}

float radToDeg(float rad) 
{
  return (rad * 360) / 6.2831;
}

