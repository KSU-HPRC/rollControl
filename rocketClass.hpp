#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <MatrixMath.h>
#include <math.h>
#include "flightplan.hpp"
#include <SPI.h>
#include <SD.h>

#ifndef _ROCKET_HPP_
#define _ROCKET_HPP_

struct ConfigParams {
    double omega;
    double moi;
    /*Insert new things as needed*/
    flightplan plan;
}

class rocket {
public:
    rocket();
    ~rocket(){};
    int parseConfig(char*, int);
    int updateSensorData(Adafruit_BNO055&, Adafruit_BMP280& /*Other Sensors*/);
    int logData();
    int updateRotMatrix();
    double getSpeed();
    double getSpeedSq();
    double getRoll();
    double getRollRate();
    double getPitch();
    double getz() {return z;};
    double* getQ() {return Q;};
private:
    // Orientation Data
    double Q[4] {0,0,0,0};
    double vQ[4] {0,0,0,0};
    double aQ[4] {0,0,0,0};

    double pitch;
    double roll;
    double rollRate;

    float R[9]{0,0,0,
               0,0,0,
               0,0,0};     // rotation matrix, stored beause it's frequently used.
    float up[3]{0,0,1};    // points "up" in the ground frame.    Placeholder is 0,0,1
    float north[3]{1,0,0}; // points "north" in the ground plane. Placeholder is 1,0,0 or 0,1,0

    // Location Data and Trajectory
    // All values should be in ground frame.
    double x;
    double y;
    double z;
    double xV;
    double yV;
    double zV;
    double xA;
    double yA;
    double zA;

    bool rollUp2Date;
    bool pitchUp2Date;
    bool rollMatrixUp2Date;

    ConfigParams model;
};

#endif
