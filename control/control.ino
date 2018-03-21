#include "rocketClass.hpp"
#include <Servo.h>
#define commsRst 6
#define controlRst 7
#define servoPin 8

//Global variables;

int flightMode;
rocket hprcRock;
Adafruit_BMP280 bmp;
Adafruit_BNO055 orient = Adafruit_BNO055(55);
Servo ailerons;
bool wireFlag = false;

//Control algorithm functions

float goalTorque(rocket &);
float deltaTorque(rocket&,float);


void setup() {
    //serialDump();
    pinMode(commsRst, OUTPUT);
    pinMode(commsRst, HIGH);
    ailerons.attach(servoPin);
    Serial.begin(9600);
    Wire.begin(75);

    Serial.print(F("Initializing SD Card..."));
    //Wire.requestFrom(commsDevice, 1);
    //Serial.print(F("Foo"));
    
    resetDev(commsRst);
    delay(1500);
    Wire.requestFrom(commsDevice,1);
    char chk = Wire.read();
    if(chk!='1'){
      Serial.print(F("No request"));
      while(1);
    }
    Serial.println(F("SD Card initialization successful."));

    Serial.print(F("Parsing Configuration File..."));
    if (hprcRock.fillModel(fpacc, commsDevice) != 0){
        Serial.print(F("CONFIG PARSE ERROR"));
        while (1);
    }
    else Serial.println(F("Config File Parsed"));

    if (!orient.begin()){
        Serial.print(F("BNO FAILURE"));
        while(1);
    }
    if (!bmp.begin()){
        Serial.println(F("BMP FAILURE"));
        while (1);
    }
    Serial.println(F("Sensors Initilized"));
    flightMode=0;
}

void loop() {
    Serial.println(F("IN LOOP"));
    //any code that needs to run every loop regardless of flightMode.
    if (hprcRock.updateSensorData(orient, bmp) == 0){

    }
    //Send Sensor Data for logging
//    hprcRock.logData();
    switch (flightMode){
        case 0 : 
            //On the ground
            break;
        case 1:
            //boost phase
            break;
        case 2: 
            //No control cost.  May be skipped, depending on what the competion rules are
            break;
        case 3:
            //Coast phase, where we control roll
            ailerons.write(hprcRock.finAngle(deltaTorque(hprcRock,goalTorque(hprcRock))));
            break;
        case 4:
            //Decent phase, initial
            break;
        case 5:
            //Decent phase, after main chute deply
            break;
        case 6:
            //on ground
            break;
    }
}

void serialDump(){
    while (Serial.available()){
        char d = Serial.read();
    }
}

void resetDev(int pin){
    digitalWrite(pin, LOW);
    delay(25);
    digitalWrite(pin, HIGH);
}

float goalTorque(rocket & BFR){
    float targetRoll=BFR.getPlan().getTargetAngle(millis())*(PI/180.0);
    return -1*BFR.getDampingConstant()*BFR.getRollRate()-BFR.getSpringConstant()*(BFR.getRoll()-targetRoll);
}

float deltaTorque(rocket& BFR,float goal){
    return BFR.getInherientTorque()-goal;
}
