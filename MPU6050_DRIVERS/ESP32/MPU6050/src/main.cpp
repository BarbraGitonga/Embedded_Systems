#include "mpu6050.h"
#include <Arduino.h>

float gx, gy, gz;
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
    MPU.gyroscope(gx, gy, gz);
    MPU.accelerometer();
    delay(1000);
}