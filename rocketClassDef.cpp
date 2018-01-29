#include "rocketClass.hpp"

rocket::rocket(){
	// Orientation Data
	pitch = 0;
	roll = 0;
	rollRate = 0;
	// Location Data and Trajectory
	// All values should be in ground frame.
	x = 0;   // Position x
	y = 0;   // Position y
	z = 0;   // Altitude
	xV = 0;  // Change in x
	yV = 0;  // Change in y
	zV = 0;  // Change in Altitude
	xA = 0;  
	yA = 0;  
	zA = 0;  
	rollUp2Date = false;
	pitchUp2Date = false;
	Adafruit_BMP280 bmp;
	Adafruit_BNO055 orient = Adafruit_BNO055(55);
    model.omega = 0;
    model.moi = 0;
}

double rocket::getSpeed(){
	return sqrt(getSpeedSq());
}
double rocket::getSpeedSq(){
	return xV*xV+yV*yV+zV*zV;
}

int rocket::updateSensorData(Adafruit_BNO055 &bno, Adafruit_BMP280 &baro){
    imu::Quaternion quat = bno.getQuat();
    Q[0] = quat.x();
    Q[1] = quat.y();
    Q[2] = quat.z();
    Q[3] = quat.w();
    z = baro.readAltitude(1013.25 /*HARDCODED, WE'LL CHANGE LATER*/);
    pitchUp2Date = false;
    rollUp2Date = false;
    return 0;
}

double rocket::getPitch(){
    /*if (!pitchUp2Date){
        float tempMatrix[9]={0};
        for(int i=0;i<9;++i) tempMatrix[i]=R[i]; //Need to copy the temp matrix
        Matrix.Multiply((float *)tempMatrix,(float *)up);
        
        pitch=acos(tempMatrix[0]*up[0]+tempMatrix[3]*up[1]+tempMatrix[6]+up[2]);
    }
    pitchUp2Date = true;*/
    return pitch;
}

double rocket::getRoll(){
    if(!rollUp2Date){
        //float tempMatrix[9]={0};
        float rocketNorth[3]={0};
        Matrix.Print((float*)rocketNorth,3,1,"n");
        //for(int i=0;i<9;++i) tempMatrix[i]=R[i]; //Need to copy the temp matrix
        Matrix.Multiply((float *)R,(float *)north,3,3,1,(float*)rocketNorth);
        Matrix.Print((float*)rocketNorth,3,1,"n");
        
        roll= atan(rocketNorth[0]/rocketNorth[1]);
    }
    rollUp2Date = true;
    return roll;
}

int rocket::updateRotMatrix(){
    double s=pow(Q[0]*Q[0]+Q[1]*Q[1]+Q[2]*Q[2]+Q[3]*Q[3],-2);
    
    R[0]=1-2*s*(Q[1]*Q[1]+Q[2]*Q[2]); R[1]=2*s*(Q[1]*Q[2]-Q[3]*Q[0]); R[2]=2*s*(Q[1]*Q[3]+Q[2]*Q[0]);
    R[3]=2*s*(Q[1]*Q[2]+Q[3]*Q[0]); R[4]=1-2*s*(Q[1]*Q[1]+Q[3]*Q[3]); R[5]=2*s*(Q[2]*Q[3]-Q[1]*Q[0]);
    R[6]=2*s*(Q[1]*Q[2]+Q[2]*Q[0]); R[7]=2*s*(Q[2]*Q[3]+Q[1]*Q[0]); R[8]=1-2*s*(Q[1]*Q[1]+Q[2]*Q[2]);

    rollMatrixUp2Date=true;

    return 0;
}

double rocket::getRollRate(){
	//Should be nearly identical to get roll, except using vQ instead of Q. 
}

int rocket::parseConfig(char* fname, int numOfParams){
    File file = SD.open(fname);
    if (file){
        int property = 0;
        char* number;
        while (file.available()){
            char ch = file.read();
            if (ch == '\n' && property < numOfParams){ /*Then we know we have a full number value*/
                double val = catod(number);
                switch (property){  /*Add new cases depending on how many properties are in the config file*/
                    case 0: model.omega = val; break;
                    case 1: model.moi = val; break;
                }
                ++property;
            }
            else if (ch == '\n' && property == numOfParams){  /*Iterated over all the properties except flight plan*/
                /*get flight plan*/
            }
            else if (!isDigit(ch) && ch != '.') continue; /*Handles #, +, -, ~, and all other non-digits with the exeption of '.'*/
            else { caAppend(number, ch); }
        }
    }
}

/*Converting a char aray to double (Found this online, dont know how well it works)*/
double catod(char* num){
    if (!num || !*num) return 0;
    double rhs = 0;
    double lhs = 0;
    int divisor = 1;
    int sign = 1;
    bool inFraction = false;
    if (*num == '-'){ ++num; sign = -1; }
    else if (*num == '+'){ ++num; }
    while (*num != '\0'){
        if (isDigit(*num)){
            if (inFraction){
                lhs = lhs*10 + (*num - '0');
                divisor *= 10;
            }
            else 
                rhs = rhs*10 + (*num - '0');
        }
        else if (*num == '.'){
            if (inFraction)
                return sign * (rhs + lhs/divisor);
            else 
                inFraction = true;
        }
        else 
            return sign * (rhs + lhs/divisor);
        ++num;
    }
    return sign * (rhs + lhs/divisor);
}

void caAppend(char* fc, int fcLen, char a){

}