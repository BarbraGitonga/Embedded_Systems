/**
 * @file mpu6050.h
 * @author Barbra Gitonga (gitongabarbra@gmail.com)
 * @brief This is a MPU6050 driver that can be used to carry out basic MPU6050 functions
 * @version 0.1
 * @date 2024-06-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MPU6050_I2C_DRIVER_H
#define MPU6050_I2C_DRIVER_H

#include <Arduino.h>

#define SELF_TEST_X         0x0D        
#define SELF_TEST_Y         0x0E
#define SELF_TEST_Z         0x0F
#define SELF_TEST_A         0x10

#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C

//acceleration readings
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

//temperature readings
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42

//gyroscope readings
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

//identity
#define WHO_AM_I            0x75
#define MPU6050_ID          0x68 //default value of who am i register

//power management 1 params
#define PWR_MGMT_1          0x6B

//Power management 2
#define PWR_MGMT_2          0x6C      



class MPU6050{
    public:
        uint8_t readRegister(uint8_t regAddress);
        uint8_t writeRegister(uint8_t regAddress, uint8_t value);
        void burstReadRegisters(uint8_t starting_reg, int bytes, int8_t* buffer);
        String identity();
        void initialize();
        int test();
        void gyroscope(float &x, float &y, float &z);
        void accelerometer(float &x, float &y, float &z);
        float temperature();
};

#endif