#include "mpu6050.h"
#include <Arduino.h>

float gx, gy, gz;
float ax, ay, az;
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
    MPU.accelerometer(ax, ay, az);
    MPU.gyroscope(gx, gy, gz);
    delay(1000);
}