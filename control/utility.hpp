#ifndef UTILLITY_HPP
#define UTILLITY_HPP
#include <Adafruit_BNO055.h>

#include"math.h"

#define constrain(x,y,z) ((y<x) ? x : ((y>z)? z: y))

union charFloatConverter{
    unsigned char b[4];
    float f;
    unsigned long l;
    int i;
};

//Print functions
void printVec(imu::Vector<3>);


//Float-byte converters for coms-control communication
float toFloat(unsigned char *);
void toChar(float, unsigned char *);
void toCharViaInt(float, unsigned char *);
void toCharViaInt(imu::Vector<3>,unsigned char *);
void toChar(imu::Vector<3>, unsigned char *);
void toChar(imu::Quaternion, unsigned char *);
void toChar(unsigned long, unsigned char *);

//FlightPlan utility
bool isDigit(char);
bool areDigits(char*, int);
int appendCharDigit(int, char);
int getNumberFromChars(char*, int);

bool isFpVital(char);
int getCaSize(char*);
char* caAppend(char*, char);
int pftoi(float&);

void toHex(unsigned char*, unsigned char*, char);
//vector math calculations
void cross(float*,float*,float*);
float vecMag(float *,char);
void normalize(float*,float*);
float dotProd(float*,float*);

//RocketClass utility

float catof(char*);

#endif
