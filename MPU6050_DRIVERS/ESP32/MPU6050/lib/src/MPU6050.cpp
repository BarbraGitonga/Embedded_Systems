#include "mpu6050.h"
#include <Wire.h>

uint8_t MPU6050::readRegister(uint8_t regAddress){
    Wire.beginTransmission(MPU6050_ID);
    Wire.write(regAddress);
    Wire.endTransmission(true);  // Do not send stop condition

    Wire.requestFrom(static_cast<uint8_t>(MPU6050_ID), static_cast<size_t>(1), true);
    return Wire.read();
}

uint8_t MPU6050::writeRegister(uint8_t regAddress, uint8_t value) {
    Wire.beginTransmission(MPU6050_ID);
    Wire.write(regAddress);
    Wire.write(value);
    Wire.endTransmission(true);
    return Wire.endTransmission(); // Return the status of the transmission
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
    
    if(identity()=="0x68"){
        Serial.println("device_found");
        //Reseting device abd signal paths
        writeRegister(PWR_MGMT_1, DEVICE_RESET);
        delay(100);
        writeRegister(SIGNAL_PATH_RESET, SENSOR_RESET);
        delay(100);

        //setting clock speed and enabling sensors
        writeRegister(PWR_MGMT_1, PWR_VAR_1);
        writeRegister(PWR_MGMT_2, 0x00); //enable all sensors

        //configuring range of sensors
        writeRegister(GYRO_CONFIG, 0x00); //full scale range of 250
        writeRegister(ACCEL_CONFIG, 0x00); //full scale range of 2g

        Serial.println("initialized");
    }
    else if(identity() == "Unknown Device"){
            Serial.println("Unknown Device");
    }
    else {
        Serial.println("No device detected");
    }
}

String MPU6050::test(){
    writeRegister(GYRO_CONFIG, 0x07); // 250dps
    writeRegister(ACCEL_CONFIG, 0x0F); //8g

    uint8_t x_ = readRegister(SELF_TEST_X);
    uint8_t y = readRegister(SELF_TEST_Y);
    uint8_t z = readRegister(SELF_TEST_Z);

    // gyroscope
    return "set";
}
  
float MPU6050::gyroscope(){

}   

float MPU6050::accelerometer(){

}

float MPU6050::temperature(){
    
}