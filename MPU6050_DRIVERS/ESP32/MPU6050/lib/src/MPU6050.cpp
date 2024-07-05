#include "mpu6050.h"
#include <Wire.h>

uint8_t MPU6050::readRegister(uint8_t regAddress){
    Wire.requestFrom(static_cast<uint8_t>(MPU6050_ID), static_cast<size_t>(1), true);
    return Wire.read();
}

uint8_t MPU6050::writeRegister(uint8_t regAddress, int value){
    Wire.beginTransmission(regAddress);
    Wire.write(value);
    Wire.endTransmission(false);
    
    return Wire.read();
}

String MPU6050::identity(){
    Wire.begin();

    Wire.beginTransmission(MPU6050_ID);
    Wire.write(WHO_AM_I);
    Wire.endTransmission(false);
    
    Wire.requestFrom(static_cast<uint8_t>(MPU6050_ID), static_cast<size_t>(1), true);

    if(Wire.available()){
        uint8_t who_am_i = Wire.read();

        if(who_am_i == MPU6050_ID){
            return "0x68";
        }
        else{
            return "Unknown Device";
        }
    }
    return "Device not detected";
}

void MPU6050::initialize(){
    Wire.begin();
    
    if(Wire.available()){
        uint8_t who_am_i = readRegister(MPU6050_ID);
        if (who_am_i == MPU6050_ID){
            writeRegister(PWR_MGMT_1, CYCLE);
            writeRegister(PWR_MGMT_1, CLKSEL);
        }
        else {
            Serial.println("Unknown Device");
        }
    }
    else {
        Serial.println("No device detected");
    }
}

    
    