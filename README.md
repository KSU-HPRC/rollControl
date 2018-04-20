# rollControl
Roll control and data logging for KSU HPRC (2017-2018)

## Libraries, SDKs, and APIs
 [Matrix Math](https://github.com/eecharlie/MatrixMath)

 [Adafruit BMP280](https://github.com/adafruit/Adafruit_BMP280_Library)

 [Adafruit BNO055](https://github.com/adafruit/Adafruit_BNO055)

 [SoftwareSerial Lib](https://www.arduino.cc/en/Reference/SoftwareSerial)

 [Servo](https://www.arduino.cc/en/Reference/Servo)

##Required tweak to core libraries:
 
Wire.h _must_ be modified such that BUFFER_LENGTH is 48 (not 32)

 
