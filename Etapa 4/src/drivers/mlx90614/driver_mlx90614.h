#ifndef _DRIVER_MLX90164_H_
#define _DRIVER_MLX90164_H_

#include "pico/stdlib.h"

// I2C bus configuration
#define MLX90614_I2C_PORT i2c0
#define MLX90614_I2C_SDA_PIN 4
#define MLX90614_I2C_SCL_PIN 5

int mlx90614_read_raw_data(uint8_t addr, uint8_t read_buffer[3]);

#endif