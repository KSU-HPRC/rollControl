#ifndef UTILLITY_CPP
#define UTILLITY_CPP
#include"utility.hpp"
#include "math.h"


char hex[16] = {'0','1','2','3','4','5','6','7','8','9',
                'A','B','C','D','E','F'};

void printVec(imu::Vector<3> v){
    Serial.print('(');
    Serial.print(v.x());
    Serial.print(',');
    Serial.print(v.y());
    Serial.print(',');
    Serial.print(v.z());
    Serial.println(')');
}

float toFloat(unsigned char * c){
    charFloatConverter converter;

    converter.b[0] = c[0];
    converter.b[1] = c[1];
    converter.b[2] = c[2];
    converter.b[3] = c[3];

    return converter.f;
}

void toChar(float in, unsigned char * c) {
    charFloatConverter converter;
    converter.f = in;

    c[0] = converter.b[0];
    c[1] = converter.b[1];
    c[2] = converter.b[2];
    c[3] = converter.b[3];
}

void toCharViaInt(float f, unsigned char * c){
    charFloatConverter converter;
    f=(f>1 || f<-1) ? (f>0 ? 1 : -1)  : f;
    converter.i=f*32767;
    c[0]=converter.b[0];
    c[1]=converter.b[1];
}

void toChar(unsigned long in, unsigned char * c) {
    charFloatConverter converter;
    converter.l = in;

    c[0] = converter.b[0];
    c[1] = converter.b[1];
    c[2] = converter.b[2];
    c[3] = converter.b[3];
}

void toChar(imu::Vector<3> v, unsigned char * c){
    toChar((float)v[0],c);
    toChar((float)v[1],c+4);
    toChar((float)v[2],c+8);
}
void toCharViaInt(imu::Vector<3> v, unsigned char * c){
    v.normalize();
    toCharViaInt((float)v[0],c);
    toCharViaInt((float)v[1],c+2);
    toCharViaInt((float)v[2],c+4);
}
void toChar(imu::Quaternion q, unsigned char * c){
    toChar((float)q.x(),c);
    toChar((float)q.y(),c+4);
    toChar((float)q.z(),c+8);
    toChar((float)q.w(),c+12);
}

bool isDigit(char c)
{
    return '0' <= c && c <= '9';
}

bool areDigits(char* c, int n)
{
    for (int i = 0; i < n; ++i)
    {
        if (!isDigit(c[i]))
        {
            return false;
        }
    }
    return true;
}

int appendCharDigit(int number, char c)
{
    if (isDigit(c))
    {
        number *= 10;
        number += c - '0';
    }
    return number;
}

int getNumberFromChars(char* c, int n)
{
    int result = 0;
    for (int i = 0; i < n; ++i)
    {
        result = appendCharDigit(result, c[i]);
    }
    return result;
}

bool isFpVital(char e){
    if (isDigit(e)) return true;
    if (e == '.') return true;
    if(e == '+') return true;
    if (e == '-') return true;
    if (e == '~') return true;
    if (e == '#') return true;
    if (e == ',') return true;
    return false;
}

int getCaSize(char* str){
    int i = 0;
    while(str[i] != '\0'){ ++i; }
    ++i;
    return i;
}

char* caAppend(char* in, char e){
    /*Appends to a char* allocated on the heap*/
    if (in == nullptr){
        in = new char[2];
        in[0] = e;
        in[1] = '\0';
        return in;
    }
    else{
        int size = getCaSize(in);
        char* temp = new char[size+2];
        int i = 0;
        for(; i < size-1; ++i){ temp[i] = in[i]; }
        delete[] in;
        in = temp;
        in[i] = e;
        in[i+1] = '\0';
        return in;
    }
}

int pftoi(float &f){
    int a = f;
    if (a != 0){
        f -= a;
        if (f < 0){f *= -1;}
        return a;
    }
    else {
        f *= 10;
        a = f;
        f -=a;
        return a;
    }
}

/*Converting a char aray to float (Found this online, dont know how well it works)*/
float catof(char* num){
    //Serial.println(num);
    if (!num || !*num) return 0;
    float rhs = 0;
    float lhs = 0;
    int divisor = 1;
    int sign = 1;
    bool inFraction = false;
    if (*num == '-'){ ++num; sign = -1; }
    else if (*num == '+'){ ++num; }
    while (*num != '\0'){
        if (isDigit(*num)){
            if (inFraction){
                divisor *= 10;
                lhs += ((float)(*num - '0'))/divisor;
            }
            else{
                rhs = rhs*10 + (*num - '0');
            }
        }
        else if (*num == '.'){
            if (inFraction)
                return sign * (rhs + lhs);
            else
                inFraction = true;
        }
        else
            return sign * (rhs + lhs);
        ++num;
    }
    return sign * (rhs + lhs);
}

#endif
