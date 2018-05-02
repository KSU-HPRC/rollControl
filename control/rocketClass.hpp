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
#define packetSize 42


class rocket {
public:
    rocket();
    ~rocket(){};
    int fillModel(int, int);
    int createRefrence(Adafruit_BNO055&, Adafruit_BMP280&,int); //Calculates the refrence frame vectors

    void beginRotation(){plan.beginRotation(lastUpdate/1000); }

    //Inflight sensor update and logging
    int updateSensorData(Adafruit_BNO055 &, Adafruit_BMP280 &);
    int sendDataComms(int);
    int sendRefComs(int,const imu::Vector<3> &,imu::Vector<3> &);

    //In flight info extraction
    float getSpeed();
    float getSpeedSq();
    float getRoll();
    float getRollRate();
    float getPitch();
    float getA_pointing();
    float getDynamicPressure();

    float getDampingConstant() { return dampingConst; }
    float getSpringConstant()  { return springConst; }
    float getRollResistance()  { return rollResist; }
    float getSystemStrength()  { return systemStrength; }

    float goalTorque();
    float inherientTorque();
    float deltaTorque(){return goalTorque()-inherientTorque();};

    int finAngle();

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

    float v;
    imu::Vector<3>a;

    //atomospheric data
    float P;
    float T;

    bool rollUp2Date;
    bool pitchUp2Date;
    bool rollMatrixUp2Date;
    bool speedUp2Date;

    float rollResist;
    float systemStrength;

    float springConst;
    float dampingConst;

    float omega;
    float calibrationPressure;
    float moi;
    flightplan plan;
};

#endif
