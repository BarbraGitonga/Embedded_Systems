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

void MPU6050::burstReadRegisters(uint8_t starting_reg, int bytes, int8_t* buffer){
    Wire.beginTransmission(MPU6050_ID);
    Wire.write(starting_reg);
    Wire.endTransmission(false);

    Wire.requestFrom(static_cast<uint8_t>(MPU6050_ID), static_cast<size_t>(bytes), true);

    for(int i=0; i<bytes; i++) {
        buffer[i] = Wire.read();
        Serial.println(buffer[i]);
    }
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
        //Reseting device and signal paths
        writeRegister(PWR_MGMT_1, DEVICE_RESET);
        delay(1000);
        writeRegister(SIGNAL_PATH_RESET, SENSOR_RESET);
        delay(1000);

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

    uint8_t x_gyro = readRegister(SELF_TEST_X);
    uint8_t y_gyro = readRegister(SELF_TEST_Y);
    uint8_t z_gyro = readRegister(SELF_TEST_Z);

    // gyroscope
    return "set";
}
  
void MPU6050::gyroscope(){
    int8_t buffer[6];
    burstReadRegisters(GYRO_XOUT_H, 6, buffer);
    int16_t gyro_x_out = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t gyro_y_out = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t gyro_z_out = (int16_t)((buffer[4] << 8) | buffer[5]);
    Serial.print("Gyro"); Serial.print(gyro_x_out);  Serial.print(gyro_y_out);  Serial.println(gyro_z_out);


    //return gyro_x_out, gyro_y_out, gyro_z_out;
}   

void MPU6050::accelerometer(){
    int8_t buffer[6];
    burstReadRegisters(ACCEL_XOUT_H, 6, buffer);
    int16_t accel_x_out = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t accel_y_out = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accel_z_out = (int16_t)((buffer[4] << 8) | buffer[5]);
    Serial.print("Accel"); Serial.print(accel_x_out); Serial.print(accel_y_out); Serial.println(accel_z_out);
}

float MPU6050::temperature(){
    int8_t buffer[2];
    burstReadRegisters(TEMP_OUT_H, 2, buffer);
    int16_t temp_out = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1]; // combined low and high bytes
    Serial.print("Temp");  Serial.println(temp_out);
    float temp_in_degrees_C = (static_cast<float>(temp_out) / 340.0f) + 36.53f; // temperature in degrees
    return temp_in_degrees_C;
}