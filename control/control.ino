#include "rocketClass.hpp"
#include <Servo.h>
#define commsRst 6
#define controlRst 7
#define servoPin 8
#define servoZero 20
#define launchPin 14
#define systemLedPin 15
#define targetAnglePin 16
#define finDeflect 20

#define red 15
#define green 16
#define blue 17

//Global variables;

int launchConnection = HIGH;

int flightMode;
rocket hprcRock;
Adafruit_BMP280 bmp;
Adafruit_BNO055 orient = Adafruit_BNO055(55);
Servo ailerons;
bool nfpValid;
bool wireFlag = false;
unsigned long lastEventTime=0;

float finAngle = 0;
//Control algorithm functions

float goalTorque(rocket &);
float deltaTorque(rocket&,float);

void setup() {
     pinMode(launchPin, OUTPUT);
     digitalWrite(launchPin, LOW);
     pinMode(launchPin, INPUT);

    pinMode(systemLedPin, OUTPUT);
    pinMode(targetAnglePin, OUTPUT);

    serialDump();
    Wire.onRequest(requestHandler);
    Wire.onReceive(receiveHandler);
    pinMode(commsRst, OUTPUT);
    pinMode(red, OUTPUT);
    pinMode(green, OUTPUT);
    pinMode(blue, OUTPUT);
    digitalWrite(commsRst, HIGH);
    delay(100);
    digitalWrite(commsRst, LOW);
    delay(100);
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
    hprcRock.createRefrence(orient, bmp,commsDevice);
    digitalWrite(systemLedPin, HIGH);
}

void loop() {
    //Serial.println(F("IN LOOP"));
    //any code that needs to run every loop regardless of flightMode.
    if (hprcRock.updateSensorData(orient, bmp) == 0){

    }
<<<<<<< HEAD

    Serial.print(F("Pitch: "));
    Serial.println(hprcRock.getPitch()*180.0/PI);
    Serial.print(F("Roll: "));
    Serial.println(hprcRock.getRoll()*180.0/PI);
    Serial.print(F("Fin Angle: "));
    Serial.println(hprcRock.finAngle());


    hprcRock.sendDataComms(commsDevice);
    //Serial.println(hprcRock.getA_pointing());
=======

<<<<<<< HEAD
>>>>>>> 6969cd9ac2d67f6204f63d14e426212d6b1f271c
=======
    Serial.print(F("Pitch: "));
    Serial.println(hprcRock.getPitch()*180.0/PI);
    Serial.print(F("\t\tRoll: "));
    Serial.println(hprcRock.getRoll()*180.0/PI);


>>>>>>> 4971f785695f4ea1d2a359a15a049906eccbc503
    //Send Sensor Data for logging
    switch (flightMode){

        case 0 :
            launchConnection = digitalRead(launchPin);
            if (launchConnection == LOW)
            {
                Serial.print("Launched");
                digitalWrite(systemLedPin, LOW);
                flightMode++;
            }
            break;
        case 1:
            //boost phase
            flightMode++;
            // Pause until after burnout.
            delay(3000);
            // Turn the LED back on when the system comes back on.
            digitalWrite(systemLedPin, HIGH);
            break;
        case 2:
            flightMode++;
            hprcRock.beginMoves(millis());
            break;
        case 3:
            //Coast phase, where we control roll
            finAngle = hprcRock.finAngle() * finDeflect;
            Serial.print("\t\t\tAngleModifier: ");
            Serial.println(finAngle);
            if(hprcRock.getPitch()<PI/4){
              //flightMode++;
              lastEventTime=millis();
              digitalWrite(systemLedPin, LOW);
            }
            if (finAngle == 0)
            {
              digitalWrite(targetAnglePin, HIGH);
            }
            else
            {
              digitalWrite(targetAnglePin, LOW);
            }
            ailerons.write(servoZero + finAngle);
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
    //delay(1000);
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
void powerLED(int finAngle){
    if (finAngle > 0){
        red.digitalWrite(0xff);
        green.digitalWrite(0x00);
        blue.digitalWrite(0x00);
    }
    else if (finAngle == 0){
        red.digitalWrite(0x00);
        green.digitalWrite(0xff);
        blue.digitalWrite(0x00);
    }
    else {
        red.digitalWrite(0x00);
        green.digitalWrite(0x00);
        blue.digitalWrite(0xff);
    }
}
