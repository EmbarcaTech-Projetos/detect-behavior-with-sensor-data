#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "driver_mpu9250.h"
#include "driver_mlx90614.h"

int main() {
    stdio_init_all();

    // Wait for the serial port to be opened by the host
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("--- Pico MPU-9250 Debugger ---\n");
    printf("Program and USB Serial are running.\n");

    i2c_init(MPU9250_I2C_PORT, 400 * 1000);
    gpio_set_function(MPU9250_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU9250_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU9250_I2C_SDA_PIN);
    gpio_pull_up(MPU9250_I2C_SCL_PIN);

    i2c_init(MLX90614_I2C_PORT, 100 * 1000); // Initialize I2C at 100kHz
    gpio_set_function(MLX90614_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MLX90614_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MLX90614_I2C_SDA_PIN);
    gpio_pull_up(MLX90614_I2C_SCL_PIN);

    int found_devices = 0;
    // This will probe the bus and tell us what devices are acknowledging.
    printf("--- Starting I2C Bus Scan ---\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    sleep_ms(1000);
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02X ", addr);
        }

        // Reserved addresses
        if (addr < 0x08 || addr > 0x77) {
            printf("  ");
            continue;
        }

        uint8_t rxdata;
        int ret = i2c_read_blocking(MLX90614_I2C_PORT, addr, &rxdata, 1, false);
        ret += i2c_read_blocking(MPU9250_I2C_PORT, addr, &rxdata, 1, false);

        if (ret < 0) {
            printf("-"); // No device found
        } else {
            printf("@"); // Device found!
            found_devices++;
        }
        
        printf(addr % 16 == 15 ? "\n" : "  ");
    }

    if (found_devices == 0) {
        printf("!!! No I2C devices found. Check wiring and power. The program will now halt. !!!\n");
        sleep_ms(100);
        return 0;
    } else {
        printf("I2C scan complete. Found %d device(s).\n", found_devices);
    }

    printf("Attempting to initialize MPU-9250...\n");
    mpu9250_init();
    printf("\nStarting 9-Axis Sensor Reading...\n\n");

    check_mpu9250_identity();

    int16_t accelerometer[3], gyroscope[3], magnetometer[3];
    for(uint8_t i=0; i<3; i++) {
        accelerometer[i] = 0;
        gyroscope[i] = 0;
        magnetometer[i] = 0;
    }
    
    uint8_t read_buffer[3];
    int result;

    // --- Main Loop ---
    while (1) {
        result = mlx90614_read_raw_data(read_buffer);
        if (result == PICO_ERROR_GENERIC) {
            printf("Error reading from sensor.\n");
        } 
        else {
            // Combine the LSB and MSB to get the raw temperature value
            uint16_t raw_temp = (uint16_t)read_buffer[1] << 8 | read_buffer[0];
            if (raw_temp & 0x8000) {
                 printf("Error flag set in temperature reading.\n");
            } else {
                // Formula: T_Kelvin = raw_value * 0.02; T_Celsius = T_Kelvin - 273.15 [cite: 1084, 1085]
                float temp_c = (raw_temp * 0.02) - 273.15;
                printf("Object Temperature: %.2f °C\n", temp_c);
            }
        }

        mpu9250_read_raw_data(accelerometer, gyroscope, magnetometer);

        printf("Accel: X=%6d, Y=%6d, Z=%6d | ", accelerometer[0], accelerometer[1], accelerometer[2]);
        printf("Gyro: X=%6d, Y=%6d, Z=%6d | ", gyroscope[0], gyroscope[1], gyroscope[2]);
        printf("Mag: X=%6d, Y=%6d, Z=%6d\n", magnetometer[0], magnetometer[1], magnetometer[2]);

        sleep_ms(500);
    }

    return 0;
}