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

#define numOfCParams 4
#define commsDevice 19
#define fpacc 5
#define numBytes 64
#define packetSize 22


class rocket {
public:
    rocket();
    ~rocket(){};
    int fillModel(int, int);
    int createRefrence(Adafruit_BNO055&, Adafruit_BMP280&); //Calculates the refrence frame vectors

    //Inflight sensor update and logging
    int updateSensorData(Adafruit_BNO055 &, Adafruit_BMP280 &);
    int sendDataComms(int);

    //In flight info extraction
    float getSpeed();
    float getSpeedSq();
    float getRoll();
    float getRollRate();
    float getPitch();

    float getDampingConstant() { return 1.0; }
    float getSpringConstant() { return 1.0; }

    float getInherientTorque() { return 0.0; } //TODO: impliment
    int finAngle(float f) { return static_cast<int>(f); }

    flightplan& getPlan(){ return plan;}
private:
    // Orientation Data
    imu::Quaternion Q;

    float pitch;
    float roll;
    float rollRate;

    unsigned long lastUpdate;
    float deltaT;

    //Ground frame basis vectors:
    imu::Vector<3> up;
    imu::Vector<3> north;
    imu::Vector<3> east;

    //Rocked basis vectors
    imu::Vector<3> pointing; //Something like (0,0,1)
    imu::Vector<3> rollRef;  //Something like (1,0,0)

    // Location Data and Trajectory
    // All values should be in ground frame.

    imu::Vector<3>v;
    imu::Vector<3>a;

    //atomospheric data
    float P;
    float T;

    bool rollUp2Date;
    bool pitchUp2Date;
    bool rollMatrixUp2Date;
    bool speedUp2Date;

    float omega;
    float calibrationPressure;
    float moi;
    flightplan plan;
};

#endif
