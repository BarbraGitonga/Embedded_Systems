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
    int tests = MPU.test();
    Serial.println(tests);
    float temp = MPU.temperature();
    Serial.println(temp);
    MPU.accelerometer(ax, ay, az);
    Serial.print("Accelerometer: "); Serial.print(ax); Serial.print(ay), Serial.println(az);
    MPU.gyroscope(gx, gy, gz);
    Serial.print("Gyroscope: "); Serial.print(gx); Serial.print(gy), Serial.println(gz);
    delay(1000);
}