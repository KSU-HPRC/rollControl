#include "rocketClass.hpp"
#include "bangbang.hpp"

#define mOverR (0.02897/8.3144598)
#define maxQ ((293.15*101300.0*mOverR)*122500.0/2)

#define omega_0 4.0

using imu::Vector;


rocket::rocket(){
    // Orientation Data
    pitch = 0;
    roll = 0;
    rollRate = 0;

    rollUp2Date = false;
    pitchUp2Date = false;
    rollMatrixUp2Date = false;
    speedUp2Date = false;

    pointing=imu::Vector<3>(1,0,0);
    rollRef=imu::Vector<3>(0,0,1);

    //systemStrength=0.00201527;
    systemStrength=100;
    rollResist=systemStrength*0.065;

    //springConst = 0.00806818;
    springConst =1;
    dampingConst; 0.01613636;

    char testPlan[] = "#3;~0901000;+901000;~2702000;";
    plan.parseFlightPlan(testPlan);
}

int rocket::createRefrence(Adafruit_BNO055 &bno, Adafruit_BMP280 &baro,int device){
    Vector<3> g=bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Vector<3> m=bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);


    //Save the refrence data;
    sendRefComs(device,g,m);

    //Get the up vector
    g.normalize();
    up=g*(-1);

    //Get north and east vectors    
    m.normalize();
    north=m-(up*m.dot(up));
    north.normalize(); //Just in case.
    east=north.cross(up);
    east.normalize(); //Just in case.
}

float rocket::getSpeed(){
    v=v+(a*deltaT);

    //return v.dot(Q.rotateVector(pointing));
    return 223;
}
float rocket::getSpeedSq(){
    float vMag=getSpeed();
    return vMag*vMag;
}

int rocket::updateSensorData(Adafruit_BNO055 &bno, Adafruit_BMP280 &baro){
    long current=micros();
    if(current-lastUpdate>10000){
        deltaT=float(current-lastUpdate)/1000000.0;
        lastUpdate=current;

        bno.getEvent(&sensorData);

        pitchUp2Date = false;
        rollUp2Date = false;
        rollMatrixUp2Date = false;
        speedUp2Date = false;

        return 1;
    }
    return 0;
}

float rocket::getPitch(){
    if (!pitchUp2Date){
        pitch = sensorData.orientation.y;
        Serial.print("Pitch: ");
        Serial.println(pitch);
    }
    pitchUp2Date = true;
    return pitch;
}

float rocket::getRoll(){
    if(!rollUp2Date){
          roll = sensorData.orientation.x;
          //Serial.print("\t\t\tRoll: ");
          //Serial.println(roll);
    }
    rollUp2Date = true;
    return roll;
}

float rocket::getRollRate(){
    getRoll();
    return rollRate;
}

float rocket::getA_pointing(){
    return a.dot(pointing);
}

float rocket::getDynamicPressure(){
    return ((P/(T+273.15))*mOverR)*getSpeedSq()/2;
}

int rocket::fillModel(int fpsize, int devName){/*
    int property = 0;
    while (property < numOfCParams){
        char* str = nullptr;
        Wire.requestFrom(commsDevice, numBytes);
        while (Wire.available()){
            char ch = Wire.read();
            if (ch == -1) break;
            str = caAppend(str, ch);
        }
        switch (property){
            case 0: omega = catof(str); break;
            case 1: moi = catof(str); break;
            case 2: calibrationPressure = catof(str); break;
            case 3: plan.parseFlightPlan(str); break;
        }
        {
            delete[] str;
            str = nullptr;
        }
        ++property;
    }*/
    return 0;
}

int rocket::sendRefComs(int device,const imu::Vector<3> & g,imu::Vector<3> & m){
    unsigned char* msg = new unsigned char[packetSize];
    unsigned char i = 0;
    toChar(g,msg);
    i+=3;
    toChar(m,msg+(i*4));
    msg[40]=2;

    Wire.beginTransmission(device);

    char j = 0;
    while (j < packetSize){
        Wire.write(msg[j]);
        ++j;
    }

    Wire.endTransmission();
    //delete[] out;
    //out = nullptr;
    delete[] msg;
    msg = nullptr;
    return 0;
}

int rocket::sendDataComms(int device){
    unsigned char* msg = new unsigned char[/*packetSize*/32];
    unsigned char i = 0;
    toChar(Q, msg);
    i += 4;
    toChar(a, msg+(i*4));
    i += 3;
    /*toChar(P, msg+(i*4));
    ++i;
    toChar(T, msg+(i*4));
    ++i;*/
    toChar(lastUpdate, msg+(i*4));
    //msg[4*(++i)] = 1;

    //Serial.println("SENDING");
    Wire.beginTransmission(device);
    //unsigned char* out = new unsigned char[(packetSize*2) + 1];
    //toHex(msg, out, packetSize);
    char j = 0;
    while (j < /*packetSize*/ 32){
        //Serial.print(out[j*2]);
        //Serial.print(out[(j*2)+1]);
        Wire.write(msg[j]);
        ++j;
    }

    Wire.endTransmission();
    //delete[] out;
    //out = nullptr;
    delete[] msg;
    msg = nullptr;
}

float rocket::goalTorque(){
    //return -getSpringConstant()*(plan.getTargetAngle(lastUpdate/1000)-getRoll())-getDampingConstant()*getRollRate();
    float result=-getSpringConstant()*(0-getRoll())-getDampingConstant()*getRollRate();
    //Serial.println(F("hi"));
    //Serial.println(result);
    return result;
}

float rocket::inherientTorque(){
    return -getRollRate()*getRollResistance()*getDynamicPressure()/getSpeed();
}

int rocket::finAngle(){
    int target = plan.getTargetAngle(millis());
    Serial.print("Fin angle:");
    Serial.println(target);
    return getFinAngle(target, getRoll());
}
