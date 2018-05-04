#include "rocketClass.hpp"

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
}

float rocket::getSpeed(){
    if(!speedUp2Date){
        v+=a[0]*deltaT;
        speedUp2Date=true;
    }
    //return v;
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

        Q = bno.getQuat(); //Takes a vector and rotates it by the same amount the BNO has since startup
        a =bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // convert a into the orignal frame
        
        T=baro.readTemperature();
        P=baro.readPressure();

        up=bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY)*(-1);
        up.normalize();

        Vector<3> m=bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        north=m-(up*m.dot(up));
        north.normalize();

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
        pitch=asin(up[0]);
    }
    pitchUp2Date = true;
    return pitch;
}

float rocket::getRoll(){
    if(!rollUp2Date){
        float oldRoll=roll;
        getPitch();

        Vector<3> ref=rollRef-up*(rollRef.dot(up))*(pitch>0 ? 1 : -1);
        ref.normalize();

        roll=(ref.dot(north.cross(up))>0) ? acos(ref.dot(north)) : 2*PI-acos(ref.dot(north));

        //Calculate roll rate:
        if(oldRoll > 7.0/4.0*PI && roll < 1.0/4.0*PI){ //Roll has likely passed from near all the way around the way around the circle through zero.
            rollRate=(roll-oldRoll+2.0*PI)/deltaT;
        } else if(roll > 7.0/4.0*PI && oldRoll < 1.0/4.0*PI){ //Roll has likely passed from barely around the circle through zero
            rollRate=(roll-oldRoll-2.0*PI)/deltaT;
        } else rollRate=(roll-oldRoll)/deltaT; //Roll has not passed through zero.
        rollUp2Date = true;
    }
    return roll;
}

float rocket::getRollRate(){
    getRoll();
    return rollRate;
}

float rocket::getA_pointing(){
    return a[0];
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

float deltaTheta(float,float);

int rocket::finAngle(){
    //Serial.println(getDynamicPressure());
    //Serial.println(goalTorque());
    float k=(5/45)*maxQ/getDynamicPressure();
    float c=4.0*k/(omega_0*omega_0);
    int raw = (180.0/PI)*(k*deltaTheta(getRoll(),plan.getTargetAngle())*(180.0/PI)+c*getRollRate());
    return constrain(-20,raw,20);
}

float deltaTheta(float a, float b){
    float res=a-b;
    if(res>PI) return res-2*PI;
    else if(res<-PI) return 2*PI+res;
    else return res;
}