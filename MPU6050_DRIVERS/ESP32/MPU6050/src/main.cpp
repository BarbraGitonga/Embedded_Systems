#include "mpu6050.h"
#include <Arduino.h>

MPU6050 MPU;
void setup(){
    Serial.begin(115200);
    String deviceID = MPU.identity();
    Serial.println("Device ID: " + deviceID);
    MPU.initialize();
}

void loop(){
    float temp = MPU.temperature();
    Serial.println(temp);
    MPU.test();
    MPU.gyroscope();
    MPU.accelerometer();
    delay(1000);
}