#include "rocketClass.hpp"
#include <Servo.h>
#define commsRst 6
#define controlRst 7
#define servoPin 8
#define servoZero 10

//Global variables;

int flightMode;
rocket hprcRock;
Adafruit_BMP280 bmp;
Adafruit_BNO055 orient = Adafruit_BNO055(55);
Servo ailerons;
bool nfpValid;
bool wireFlag = false;
unsigned long lastEventTime=0;

//Control algorithm functions

float goalTorque(rocket &);
float deltaTorque(rocket&,float);

void setup() {
    serialDump();
    Wire.onRequest(requestHandler);
    Wire.onReceive(receiveHandler);
    pinMode(commsRst, OUTPUT);
    pinMode(commsRst, HIGH);
    ailerons.attach(servoPin);
    ailerons.write(servoZero); 
    Serial.begin(57600);
    Wire.begin(75);
    
    Serial.print(F("Initializing SD Card..."));
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

    delay(3000);
    //hprcRock.createRefrence(orient, bmp,commsDevice);
}

void loop() {
    //Serial.println(F("IN LOOP"));
    //any code that needs to run every loop regardless of flightMode.
    if (hprcRock.updateSensorData(orient, bmp) == 0){

    }
    
    Serial.print(F("Pitch: "));
    Serial.println(hprcRock.getPitch()*180.0/PI);
    Serial.print(F("Roll: "));
    Serial.println(hprcRock.getRoll()*180.0/PI);
    Serial.println(hprcRock.finAngle());
    
    
    hprcRock.sendDataComms(commsDevice);
    //Serial.println(hprcRock.getA_pointing());
    //Send Sensor Data for logging
    switch (flightMode){
        
        case 0 :
            //prelaunch
            if(hprcRock.getA_pointing()>20) {
              flightMode++;
              lastEventTime=millis();
            }
            break;
        case 1:
            //boost phase
            if(hprcRock.getA_pointing()<10){
              flightMode++;
              lastEventTime=millis();
            }
            break;
        case 2:
            if(millis()-lastEventTime>=250){
              flightMode++;
              lastEventTime=millis();
            }
            break;
        case 3:
            //Coast phase, where we control roll
            ailerons.write(servoZero+5);
            //Serial.print(F("Fin a"));
            //Serial.println(hprcRock.finAngle());
            //ailerons.write(servoZero+hprcRock.finAngle());
            if(millis()-lastEventTime>=3000){
              flightMode++;
              lastEventTime=millis();
            }
            break;
        case 4:
            //Decent phase, initial
            ailerons.write(servoZero);
            break;
        case 5:
            //Decent phase, after main chute deply
            break;
        case 6:
            //on ground
            break;
    }
    //For debug
    delay(1000);
}

void receiveHandler(int bytesReceived){
    switch(flightMode){
        case 0: newFlightPlan();
    }
}

void requestHandler(){
    switch(flightMode){
        case 0: {
            if (nfpValid) sendAck();
            else sendErr();
        }
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

void newFlightPlan(){
    char* potfp = nullptr;
    for (int i = 0; Wire.available(); ++i){
        potfp = caAppend(potfp, Wire.read());
    }
    Serial.println(F("GOT NEW FP!"));
    flightplan nfp;
    nfp.parseFlightPlan(potfp);
    if (nfp.validFlightPlan()){
        hprcRock.getPlan() = nfp;
        nfpValid = true;
    }
    else nfpValid = false;
}

void sendAck(){ Wire.write('0'); }
void sendErr(){ Wire.write('1'); }
