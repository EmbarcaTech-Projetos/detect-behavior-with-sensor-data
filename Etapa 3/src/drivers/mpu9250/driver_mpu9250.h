#ifndef _DRIVER_MPU9250_H_
#define _DRIVER_MPU9250_H_

#define MPU9250_I2C_PORT i2c1
#define MPU9250_I2C_SDA_PIN 2
#define MPU9250_I2C_SCL_PIN 3
#define MPU9250_ADDR                0x68
#define MPU9250_REG_PWR_MGMT_1      0x6B
#define MPU9250_REG_INT_PIN_CFG     0x37
#define MPU9250_REG_ACCEL_XOUT_H    0x3B
#define MPU9250_REG_GYRO_XOUT_H     0x43
#define AK8963_ADDR         0x0C
#define AK8963_REG_CNTL1    0x0A
#define AK8963_REG_HXL      0x03

void mpu9250_init();
void mpu9250_read_raw_data(int16_t accel[3], int16_t gyro[3], int16_t mag[3]);
void check_mpu9250_identity();

#endif