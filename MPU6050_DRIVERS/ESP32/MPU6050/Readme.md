# MPU6050 driver in Arduino.

 This is a driver that can be used on MPU6050 / 6000 and MPU9250 to read accelerometer, gyroscope and temperature values from the sensor.

## How to use

1. Download the library  folder that holds the MPU6050 driver.

```
#include "mpu6050.h"
#include <Arduino.h>

float gx, gy, gz;
float ax, ay, az;

MPU6050 MPU;
void setup(){
    Serial.begin(115200);
    String deviceID = MPU.identity(); // shows the identity of the device
    Serial.println("Device ID: " + deviceID);
    MPU.initialize(); // initializes the sensor
}

void loop(){
    // int tests = MPU.test(); // use this to run tests
    // Serial.println(tests);
    float temp = MPU.temperature(); // read temperature
    Serial.println(temp);
    MPU.accelerometer(ax, ay, az); // read accelerometer values into the variables
    Serial.print("Accelerometer: "); Serial.print(ax); Serial.print(ay), Serial.println(az);
    MPU.gyroscope(gx, gy, gz); // read gyroscope values into the variables
    Serial.print("Gyroscope: "); Serial.print(gx); Serial.print(gy), Serial.println(gz);
    delay(1000);
}
```
