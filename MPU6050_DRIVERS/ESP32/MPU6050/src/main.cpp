#include "mpu6050.h"
#include <Arduino.h>

void setup(){
    Serial.begin(115200);
}

void loop(){
    MPU6050 MPU;
    //Serial.println(MPU.identity());
    String deviceID = MPU.identity();
    Serial.println("Device ID: " + deviceID);
    delay(1000);
}