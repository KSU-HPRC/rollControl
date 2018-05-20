#include "utility.hpp"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define configFile "ROCKETCF.TXT"
#define flightLog "FLOGGER1.TXT"
#define sdPin 10
#define fpacc 5
#define controlDevice 75
#define packetSize 12

File configf;
File logger;
int cmdSqnc = 0;
unsigned char* data = new unsigned char[packetSize];
char* fp2 = nullptr;
bool keepListening = false;
char* inData = nullptr;
int command = -1;

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
        char inc = Serial.read();
    delay(1);
    if (inc != -1){
        if (inc != '\0') inData = caAppend(inData, inc);
        else {
            inData = caAppend(inData, '\0');
            Serial.println(inData[2]);
            if (inData[2] == '~') command = 0;
            else if (inData[2] == '`') command = 1;
            else command = -1;
            if (inData[0] != 'k' || inData[1] != 's') command = -1;

            if (command == 0){
                int i = 3; //security check skipped
                char* nfp = nullptr;
                while (keepListening){
                    if (inData[i] == '\0'){ //entire FP received
                        int j = 0;
                        while (nfp[j] != '\0'){
                            Serial.print(nfp[j]);
                            ++j;
                        }
                        keepListening = false;
                    }
                    else if (inData[i] == -1) continue;
                    else nfp = caAppend(nfp, inData[i]);
                }
            }
            else if (command == 1){
                Serial.println("CALC");
                int one = inData[3] - '0';
                int two = inData[5] - '0';
                int three = inData[7] - '0';
                int answer;
                if (inData[4] == '+') answer = one + two;
                else if(inData[4] == '*') answer = one * two;
                else if(inData[4] == '-') answer = one - two;
                if (inData[6] == '+') answer += three;
                else if (inData[6] == '*') answer *= three;
                else if (inData[6] == '-') answer -= three;
                Serial.print("ks");
                Serial.print(answer);
                delete[] inData;
                inData = nullptr;
            }
            else {} //do nothing
        }
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

void transmitXbee(char* str, int len){
    char i = 0;
    while (i < len){
        Serial.print(str[i]);
        ++i;
    }
    Serial.println();
}

void serialDump(){
    while (Serial.available()){
        char d = Serial.read();
    }
}
