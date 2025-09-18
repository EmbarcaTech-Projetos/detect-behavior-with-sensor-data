#include "driver_mlx90614.h"
#include "hardware/i2c.h"

// MLX90614 Sensor Configuration
const uint8_t MLX90614_ADDR = 0x5A;
const uint8_t MLX90614_REG_TOBJ1 = 0x07;

int mlx90614_read_raw_data(uint8_t addr, uint8_t read_buffer[3]) {
    // Write the address of the temperature register we want to read (TOBJ1)
    i2c_write_blocking(MLX90614_I2C_PORT, addr, &MLX90614_REG_TOBJ1, 1, true);
        
    // Read 3 bytes: Temperature LSB, Temperature MSB, and PEC (Packet Error Code)
    int result = i2c_read_blocking(MLX90614_I2C_PORT, addr, read_buffer, 3, false);
    return result; 
}