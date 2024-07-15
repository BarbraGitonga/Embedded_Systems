#include "mpu6050.h"
#include <Arduino.h>

MPU6050 MPU;
void setup(){
    Serial.begin(115200);
    MPU.initialize();
}

void loop(){
    //Serial.println(MPU.identity());
    String deviceID = MPU.identity();
    Serial.println("Device ID: " + deviceID);
    delay(5000);
}