#include "hardware/i2c.h"
#include "pico/time.h"
#include "driver_mpu9250.h"
#include <stdio.h>

void mpu9250_init() {
    uint8_t buffer[2];

    // 1. Wake up MPU9250
    buffer[0] = MPU9250_REG_PWR_MGMT_1;
    buffer[1] = 0x00;
    i2c_write_blocking(MPU9250_I2C_PORT, MPU9250_ADDR, buffer, 2, false);

    // 2. Enable I2C Bypass Mode to access AK8963 magnetometer
    buffer[0] = MPU9250_REG_INT_PIN_CFG;
    buffer[1] = 0x02;
    i2c_write_blocking(MPU9250_I2C_PORT, MPU9250_ADDR, buffer, 2, false);

    // --- CRITICAL STEP ---
    // 3. Add a delay to allow the bypass multiplexer to settle.
    sleep_ms(10); 
    
    // 4. Robust AK8963 Initialization Sequence
    // Power down magnetometer
    buffer[0] = AK8963_REG_CNTL1;
    buffer[1] = 0x00;
    i2c_write_blocking(MPU9250_I2C_PORT, AK8963_ADDR, buffer, 2, false);
    sleep_ms(10);

    // Enter Fuse ROM access mode
    buffer[0] = AK8963_REG_CNTL1;
    buffer[1] = 0x0F;
    i2c_write_blocking(MPU9250_I2C_PORT, AK8963_ADDR, buffer, 2, false);
    sleep_ms(10);

    // Power down magnetometer again
    buffer[0] = AK8963_REG_CNTL1;
    buffer[1] = 0x00;
    i2c_write_blocking(MPU9250_I2C_PORT, AK8963_ADDR, buffer, 2, false);
    sleep_ms(10);

    // Set continuous measurement mode 2 (100Hz) and 16-bit output
    buffer[0] = AK8963_REG_CNTL1;
    buffer[1] = 0x16; // 0b00010110
    i2c_write_blocking(MPU9250_I2C_PORT, AK8963_ADDR, buffer, 2, false);
    sleep_ms(10);
}

void mpu9250_read_raw_data(int16_t accel[3], int16_t gyro[3], int16_t mag[3]) {
    uint8_t buffer[6];
    uint8_t reg;
    reg = MPU9250_REG_ACCEL_XOUT_H;
    i2c_write_blocking(MPU9250_I2C_PORT, MPU9250_ADDR, &reg, 1, true);
    i2c_read_blocking(MPU9250_I2C_PORT, MPU9250_ADDR, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }
    reg = MPU9250_REG_GYRO_XOUT_H;
    i2c_write_blocking(MPU9250_I2C_PORT, MPU9250_ADDR, &reg, 1, true);
    i2c_read_blocking(MPU9250_I2C_PORT, MPU9250_ADDR, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }
    uint8_t mag_buffer[7];
    reg = AK8963_REG_HXL;
    i2c_write_blocking(MPU9250_I2C_PORT, AK8963_ADDR, &reg, 1, true);
    i2c_read_blocking(MPU9250_I2C_PORT, AK8963_ADDR, mag_buffer, 7, false);
    mag[0] = (mag_buffer[1] << 8) | mag_buffer[0];
    mag[1] = (mag_buffer[3] << 8) | mag_buffer[2];
    mag[2] = (mag_buffer[5] << 8) | mag_buffer[4];
}

void check_mpu9250_identity() {
    // The WHO_AM_I register address for the MPU-9250.
    const uint8_t WHO_AM_I_REG = 0x75;
    // The expected value for a genuine MPU-9250.
    const uint8_t EXPECTED_VALUE = 0x71;

    uint8_t who_am_i_value;

    // First, write the address of the register we want to read.
    i2c_write_blocking(MPU9250_I2C_PORT, MPU9250_ADDR, &WHO_AM_I_REG, 1, true);

    // Now, read the 1-byte value from that register.
    int result = i2c_read_blocking(MPU9250_I2C_PORT, MPU9250_ADDR, &who_am_i_value, 1, false);

    printf("\n--- MPU-9250 Identity Check ---\n");
    if (result < 0) {
        printf("Error: Could not read WHO_AM_I register. Check wiring and address.\n");
    } else {
        printf("Reading WHO_AM_I register... Device ID: 0x%02X\n", who_am_i_value);
        if (who_am_i_value == EXPECTED_VALUE) {
            printf("Result: This appears to be a genuine MPU-9250. ✅\n");
            printf("It should have a working 3-axis magnetometer.\n");
        } else {
            printf("Result: This is NOT a genuine MPU-9250 (ID is not 0x71). ❌\n", who_am_i_value);
            printf("It might be a clone or a different sensor (like an MPU-6050) and likely lacks a magnetometer.\n");
        }
    }
    printf("----------------------------------\n\n");
}