#include "utility.hpp"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define configFile "ROCKETCF.TXT"
#define flightLog "FLOGGER1.TXT"
#define sdPin 10
#define fpacc 5
#define controlDevice 75
#define packetSize 42

File configf;
File logger;
int cmdSqnc = 0;
unsigned char* data = new unsigned char[packetSize];
char* fp2 = nullptr;
bool keepListening = false;

void serialEvent() {
  if (Serial.available()) {
    Serial.write(Serial.read());
  }
}

void setup(){
    serialDump();
    SD.begin(sdPin);
    Serial.begin(57600);
    Wire.begin(19);

    Serial.print(F("Waiting For Instruction..."));
    Wire.onRequest(requestHandler);
    Wire.onReceive(receiveHandler);
}

void loop(){
    Serial.println("LOOPING");
    while (keepListening){
        char inc = Serial.read();
        delay(1);
        if (inc == '\0'){ // entire FP received
            int i = 0;
            while(fp2[i] != '\0'){
                Wire.write(fp2[i]);
                ++i;
            }
            Wire.requestFrom(controlDevice, 1);
            if (Wire.read() == '0'){
                Serial.println(F("VALID FLIGHT PLAN"));
                keepListening = false;
                break;
            }
            else {
                Serial.println(F("INVALID FLIGHT PLAN. TRY AGAIN"));
                delete[] fp2;
                fp2 = nullptr;
                continue;
            }
        }
        else if (inc == -1) continue;
        else fp2 = caAppend(fp2, inc);
    }
}

void requestHandler(){
    switch (cmdSqnc){
        case 0: ackSD(); break;
        case 1: sendParam(); break;
        default: {}
    }
}

void receiveHandler(int bytesReceived){
    char i = 0;
    while(Wire.available()){
        data[i] = Wire.read();
        //Serial.print(data[i]);
        ++i;
    }
    unsigned char* out = new unsigned char[(packetSize*2)+1];
    toHex(data, out, packetSize);
    logSD(out, (packetSize*2)+1);
    transmitXbee(out, (packetSize*2)+1);
    delete[] out;
    out = nullptr;
}

void ackSD(){
    Serial.print(F("Initilizing SD card..."));
    configf = SD.open(configFile);
    if (configf){
        sendAck();
        Serial.println(F("SD Card Init Success!"));
        ++cmdSqnc;
    }
    else Serial.println(F("SD Card Init Failure!"));
}

void sendParam(){
    Serial.print(F("Parsing a parameter..."));
    if (configf){
        char* str = nullptr;
        while (1){
            char ch = configf.read();
            if (ch == -1) {
                Serial.println(F("Done Parsing"));
                ++cmdSqnc;
                break;
            }
            else if (ch == '\n'){
                Serial.print(F("Sending Parameter..."));
                Wire.write(str);
                delete[] str;
                str = nullptr;
                break;
            }
            else if (isFpVital(ch)){str=caAppend(str, ch);}
        }
    }
}

void sendAck(){
    Wire.write('1');
}

void logSD(unsigned char* str, int len){
    logger = SD.open(flightLog, FILE_WRITE);
    char i = 0;
    while (i < len){
        logger.write(str[i]);
        ++i;
    }
    logger.write('\n');
    logger.close();
}

void transmitXbee(unsigned char* str, int len){
    char i = 0;
    while (i < len){
        Serial.print(str[i]);
        ++i;
    }
}

void serialDump(){
    while (Serial.available()){
        char d = Serial.read();
    }
}
