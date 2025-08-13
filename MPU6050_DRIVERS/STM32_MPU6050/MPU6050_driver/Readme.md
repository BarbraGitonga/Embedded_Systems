# MPU6050 driver for stm32F7xx

This driver is an MPU6050 driver for the STM32F7xx series.

## Setting up your STM32CubeIDE.

I am using the STM32F746 DISCO board.
On the .ioc, ensure you have the following setup:
- I2C (of your choice, I am using I2C1) has the following setup:
    - Both SDA and SCL pins ought to be in pull-up mode in `GPIO Settings`.
    - Standard speed.
- NVIC
    - Incase you want to use interrupts, set a free pin on the board to be your interrupt pin.
    - Set the pin as GPIO_EXT mode.
    - Set the pin in NVIC to be enabled.

## Using the driver.
```cpp
MPU600_Data mpu_data; // struct that holds data on accelerometer, gyroscope and temperature and your i2c handler.
MPU6050 mpu;
main{
  // include all generated code
  // user code
 
  mpu.initialize(mpu_data, &hi2c1); // use the i2c handler as what you chose in .ioc, ie I2C1 -> hi2c1
  // end of user code main
}

while(1) {
  mpu.accelerometer(&mpu_data); // stores accelerometer values in mpu_data.acc_mps2
  mpu.gyroscope(&mpu_data); // stores gyroscope in values of mpu_data.gyro_rad
  mpu.temperature(&mpu_data); // stores temp values in mpu_data.temp
}
```
