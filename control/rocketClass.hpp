#ifndef _ROCKET_HPP_
#define _ROCKET_HPP_

#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <MatrixMath.h>
#include "flightplan.hpp"
#include "utility.hpp"

#define numOfCParams 3
#define commsDevice 19
#define fpacc 5
#define numBytes 64

#define PI 2*asin(1)

class rocket {
public:
    rocket();
    ~rocket(){};
    int fillModel(int, int);
    int updateSensorData(Adafruit_BNO055&, Adafruit_BMP280&);
    int logData(char*, int);
    int updateRotMatrix();

    float getSpeed();
    float getSpeedSq();
    float getRoll();
    float getRollRate();
    float getPitch();
    float getz() {return z;};
    float* getQ() {return (float*)Q;};

    float getDampingConstant() { return 1.0; }
    float getSpringConstant() { return 1.0; }

    float getInherientTorque() { return 0.0; } //TODO: impliment
    int finAngle(float f) { return static_cast<int>(f); }

    const flightplan& getPlan(){ return plan;}
private:
    // Orientation Data
    float Q[4] {0,0,0,0};

    float pitch;
    float roll;
    float rollRate;

    long lastUpdate;
    int deltaT;

    float R[9]{0,0,0,
               0,0,0,
               0,0,0};     // rotation matrix, stored beause it's frequently used.
    float up[3]{0,0,1};    // points "up" in the ground frame.    Placeholder is 0,0,1.  Must be a unit vector
    float north[3]{1,0,0}; // points "north" in the ground plane. Placeholder is 1,0,0 or 0,1,0

    // Location Data and Trajectory
    // All values should be in ground frame.

    float v[3]{0,0,0};
    float a[3]{0,0,0};
    float aNRocketFrame[3]{0,0,0};

    //atomospheric data
    float P;
    float T;

    bool rollUp2Date;
    bool pitchUp2Date;
    bool rollMatrixUp2Date;
    bool speedUp2Date;

    float omega;
    float moi;
    flightplan plan;
};

#endif
