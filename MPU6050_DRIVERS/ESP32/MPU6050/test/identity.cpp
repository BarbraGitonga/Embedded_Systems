#include "mpu6050.h"
#include <Arduino.h>

void setup(){
    Serial.begin(115200);
    MPU6050 MPU;
    String deviceID = MPU.identity();
    Serial.println("Device ID: " + deviceID);
}

void loop(){

}