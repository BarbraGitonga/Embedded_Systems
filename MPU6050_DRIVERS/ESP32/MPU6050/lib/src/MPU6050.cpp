#include "mpu6050.h"
#include <Wire.h>

/**
 * @brief Reads froma register
 * 
 * @param regAddress - register to read from
 * @return uint8_t data that has been read by int.
 */
uint8_t MPU6050::readRegister(uint8_t regAddress){
    Wire.beginTransmission(MPU6050_ID);
    Wire.write(regAddress);
    Wire.endTransmission(true);  // Do not send stop condition

    Wire.requestFrom(static_cast<uint8_t>(MPU6050_ID), static_cast<size_t>(1), true);
    return Wire.read();
}

/**
 * @brief Writes a specific value to the register given
 * 
 * @param regAddress - register to write to from
 * @param value  - what to be written into.
 * @return uint8_t 
 */
uint8_t MPU6050::writeRegister(uint8_t regAddress, uint8_t value) {
    Wire.beginTransmission(MPU6050_ID);
    Wire.write(regAddress);
    Wire.write(value);
    Wire.endTransmission(true);
    return Wire.endTransmission(); // Return the status of the transmission
}

/**
 * @brief Reads data from multiple registers at once
 * 
 * @param starting_reg - the first register  to start from
 * @param bytes - number of  bytes to be gotten starting from the
 * @param buffer - pass a variable thats your buffer
 */
void MPU6050::burstReadRegisters(uint8_t starting_reg, int bytes, int8_t* buffer){
    Wire.beginTransmission(MPU6050_ID);
    Wire.write(starting_reg);
    Wire.endTransmission(false);

    Wire.requestFrom(static_cast<uint8_t>(MPU6050_ID), static_cast<size_t>(bytes), true);

    for(int i=0; i<bytes; i++) {
        buffer[i] = Wire.read();
    }
}

/**
 * @brief - identifies the device
 * 
 * @return String 
 */
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

/**
 * @brief - initialization of the device
 * 
 */
void MPU6050::initialize(){
    Wire.begin();
    String identity_d = identity();

    if(identity_d=="0x68"){

        //setting clock speed and enabling sensors
        writeRegister(PWR_MGMT_1, 0x00); //reset the MPU6050
        writeRegister(PWR_MGMT_2, 0x00); //enable all sensors

        //configuring range of sensors
        writeRegister(GYRO_CONFIG, 0x00); //full scale range of 250
        writeRegister(ACCEL_CONFIG, 0x00); //full scale range of 2g

        Serial.println("initialized");
    }
    else if(identity_d == "Unknown Device"){
            Serial.println("Unknown Device");
    }
    else {
        Serial.println("No device detected");
    }
}

void MPU6050::test(){
    uint8_t xg_test = readRegister(SELF_TEST_X);
    uint8_t yg_test = readRegister(SELF_TEST_Y);
    uint8_t zg_test = readRegister(SELF_TEST_Z);
    
    Serial.print("Gyro Self Test - X: 0x"); Serial.print(xg_test, HEX);
    Serial.print(" Y: 0x"); Serial.print(yg_test, HEX);
    Serial.print(" Z: 0x"); Serial.println(zg_test, HEX);
}

/**
 * @brief 
 * 
 */
void MPU6050::gyroscope(float &x, float &y, float &z){
    int8_t buffer[6];
    burstReadRegisters(GYRO_XOUT_H, 6, buffer);
    Serial.println();
    int16_t gyro_x_out = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t gyro_y_out = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t gyro_z_out = (int16_t)((buffer[4] << 8) | buffer[5]);

    float scale = 250.0 / 32768.0; // convert raw values to 
    x = gyro_x_out * scale;
    y = gyro_y_out * scale;
    z = gyro_z_out * scale;
}   

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 */
void MPU6050::accelerometer(float &x, float &y, float &z){
    int8_t buffer[6];
    burstReadRegisters(ACCEL_XOUT_H, 6, buffer);
    int16_t accel_x_out = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t accel_y_out = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accel_z_out = (int16_t)((buffer[4] << 8) | buffer[5]);

    float scale = 2.0 / 32768.0;
    x = accel_x_out * scale;
    y = accel_x_out * scale;
    z = accel_x_out * scale;
    
}

/**
 * @brief 
 * 
 * @return float 
 */
float MPU6050::temperature(){
    int8_t buffer[2];
    burstReadRegisters(TEMP_OUT_H, 2, buffer);
    int16_t temp_out = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1]; // combined low and high bytes
    float temp_in_degrees_C = (static_cast<float>(temp_out) / 340.0f) + 36.53f; // temperature in degrees
    return temp_in_degrees_C;
}