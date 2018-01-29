#include "rocketClass.hpp"
#define servoPin 8
#define sdPin 4

int flightMode;
rocket hprcRock;

Adafruit_BMP280 bmp;
Adafruit_BNO055 orient = Adafruit_BNO055(55);

Servo ailerons;
File config;

void setup() {
    ailerons.attach(servoPin);
    Serial.begin(9600);
    /*
    Serial.print("Initializing SD Card...");
    if (!SD.begin(sdPin)){
        Serial.println("Initialization Failed!");
        while (1);
    }
    //Attempting to Integrate SD. Ignore for now
    Serial.println("SD Card initialization successful.");
    */
    if (!orient.begin()){
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    Serial.println("BNO055 initialization successful.");
    if (!bmp.begin()){  
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        while (1);
    }
    Serial.println("BMP280 initialization successful.");
    flightMode=0;
}

void loop() {
    //any code that needs to run every loop regardless of flightMode.
    if (hprcRock.updateSensorData(orient, bmp) == 0){
        double* qt = hprcRock.getQ();

        Serial.println("PRINTING QUATERION:");
        Serial.println(qt[0]);
        Serial.println(qt[1]);
        Serial.println(qt[2]);
        Serial.println(qt[3]);

        Serial.println("PRINTING ALTITUDE:");
        Serial.println(hprcRock.getz());
    }
    Serial.println(hprcRock.getRoll());
    switch (flightMode){
        case 0 : 
          //On the ground
          break;
        case 1:
          //boost phase
          break;
        case 2:
          //Coast phase, where we control roll
          break;
        case 3:
          //Decent phase, initial
          break;
        case 4:
          //Decent phase, after main chute deply
          break;
        case 5:
          //on ground
          break;
    }
}
