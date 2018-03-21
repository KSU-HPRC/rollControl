#include "rocketClass.hpp"

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

    pointing=imu::Vector<3>(0,0,1);
    rollRef=imu::Vector<3>(1,0,0);


    omega = 0;
    moi = 0;
}

int rocket::createRefrence(Adafruit_BNO055 &bno, Adafruit_BMP280 &baro){
    Vector<3> g=bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    g.normalize();
    up=g*-1;

    Vector<3> m=bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    m.normalize();
    north=m-(up*m.dot(up));
    north.normalize(); //Just in case.
    east=north.cross(up);
    east.normalize; //Just in case.
}

float rocket::getSpeed(){
    return sqrt(getSpeedSq);
}
float rocket::getSpeedSq(){
	if(!speedUp2Date){
        float inverseR[9];
        Matrix.Copy((float *)R,3,3,(float *)inverseR);
        Matrix.Invert((float *)inverseR,3);
        Matrix.Multiply((float *)inverseR,(float *)aNRocketFrame,3,3,1,(float * )a);
        
        v[0]+=(1000000.0/((float)deltaT))*a[0];
        v[1]+=(1000000.0/((float)deltaT))*a[1];
        v[2]+=(1000000.0/((float)deltaT))*a[2];   
    }

    updateRotMatrix()
    float rocketUp[3]={0};
    Matrix.Multiply((float *)R,(float *)up,3,3,1,(float*)rocketUp)

    return dotProd((float *)v,(float *) rocketUp);
}

int rocket::updateSensorData(Adafruit_BNO055 &bno, Adafruit_BMP280 &baro){
    long current=millis();
    if(current-lastUpdate>10){
        deltaT=float(current-lastUpdate)/1000000.0;
        lastUpdate=current;

        Q = bno.getQuat();
        a = Q.rotateVector(bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL));
        
        T=baro.readTemperature();
        P=baro.readPressure()

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
        pitch=asin((Q.rotateVector(pointing)).dot(up));
    }
    pitchUp2Date = true;
    return pitch;
}

float rocket::getRoll(){
    if(!rollUp2Date){
        float oldRoll=roll;
        Vector<3> ref;
        getPitch()


        if(pitch>PI/2.0-.01){ // Breaks for the rocket pointed stragith down.  Hopefully won't be an issue.

            Vector<3> axis=pointing.cross(Q.rotateVector(pointing));
            float angle=asin(axis.magnitude());
            axis.normalize();
            imu::Quaternion toVertical;
            toVertical.fromAngleAxis(axis,angle);
            ref=toVertical.rotateVector(Q.rotateVector(rollRef));

        } else {
            ref=toVertical.rotateVector(rollRef);
        }

       if(east.dot(ref)>0){
           roll=north.dot(ref);
       } else {
           roll=2*PI+north.dot(ref);
       }

        //Calculate roll rate:
        if(oldRoll > 7.0/4.0*PI && roll < 1.0/4.0*PI){ //Roll has likely passed from near all the way around the way around the circle through zero.
            rollRate=1000000.0*(roll-oldRoll+2.0*PI)/float(deltaT);
        } else if(roll > 7.0/4.0*PI && oldRoll < 1.0/4.0*PI){ //Roll has likely passed from barely around the circle through zero
            rollRate=1000000.0*(roll-oldRoll)/float(deltaT);
        } else rollRate=1000000.0*(roll-oldRoll-2.0*PI)/float(deltaT); //Roll has not passed through zero.
    }
    rollUp2Date = true;
    return roll;
}

float rocket::getRollRate(){
    getRoll();
	return rollRate;
}

int rocket::fillModel(int fpsize, int devName){
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
            case 2: plan.parseFlightPlan(str); break;
        }
        {
          delete[] str;
          str = nullptr;
        }
        ++property;
    }
    return 0;
}

int rocket::logData(char* fname, int floatSize){
    
}
