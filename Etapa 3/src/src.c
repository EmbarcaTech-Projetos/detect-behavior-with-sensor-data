#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C bus configuration from your driver_mlx90614.h
#define MLX90614_I2C_PORT i2c0
#define MLX90614_I2C_SDA_PIN 4
#define MLX90614_I2C_SCL_PIN 5

// Default MLX90614 address
const uint8_t MLX90614_DEFAULT_ADDR = 0x5A;

// EEPROM address for changing the I2C slave address
const uint8_t MLX90614_REG_EEPROM_ADDR = 0x2E;

/**
 * @brief Calculates the SMBus CRC-8 Packet Error Code (PEC).
 * @param data Pointer to the data array.
 * @param len  Number of bytes in the data array.
 * @return The calculated 8-bit PEC.
 */
uint8_t calculate_pec(uint8_t *data, int len) {
    uint8_t pec = 0;
    for (int i = 0; i < len; i++) {
        pec ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (pec & 0x80) {
                pec = (pec << 1) ^ 0x07;
            } else {
                pec <<= 1;
            }
        }
    }
    return pec;
}

/**
 * @brief Writes a new I2C address to the sensor's EEPROM.
 * @note  Only one MLX90614 should be on the bus when this is run.
 * The sensor MUST be power-cycled for the new address to take effect.
 * @param new_addr The new 7-bit I2C address (0x01-0x7F).
 */
void mlx90614_set_address(uint8_t new_addr) {
    if (new_addr < 0x01 || new_addr > 0x7F) {
        printf("Error: Invalid I2C address provided.\n");
        return;
    }

    uint8_t pec_buffer[4]; // Buffer for PEC calculation
    uint8_t command_buffer[4]; // Buffer for I2C write command

    // The data for PEC calculation includes the slave address (shifted left by 1)
    pec_buffer[0] = MLX90614_DEFAULT_ADDR << 1;

    // === Step 1: Erase the EEPROM address cell by writing 0x0000 ===
    command_buffer[0] = MLX90614_REG_EEPROM_ADDR; // Command to write to EEPROM address register
    command_buffer[1] = 0x00; // Data Low Byte
    command_buffer[2] = 0x00; // Data High Byte

    // Calculate PEC for the erase command
    pec_buffer[1] = command_buffer[0];
    pec_buffer[2] = command_buffer[1];
    pec_buffer[3] = command_buffer[2];
    command_buffer[3] = calculate_pec(pec_buffer, 4);

    printf("Attempting to erase EEPROM address cell...\n");
    // Write 4 bytes: [Command, Data_LSB, Data_MSB, PEC]
    int result = i2c_write_blocking(MLX90614_I2C_PORT, MLX90614_DEFAULT_ADDR, command_buffer, 4, false);
    if (result < 0) {
        printf("Error: Failed to send erase command. Sensor not found at 0x%X?\n", MLX90614_DEFAULT_ADDR);
        return;
    }
    sleep_ms(10); // EEPROM writes are slow, give it time

    // === Step 2: Write the new address to the EEPROM cell ===
    command_buffer[0] = MLX90614_REG_EEPROM_ADDR; // Command
    command_buffer[1] = new_addr; // Data Low Byte (the new address)
    command_buffer[2] = 0x00; // Data High Byte (must be 0)

    // Calculate PEC for the write command
    pec_buffer[1] = command_buffer[0];
    pec_buffer[2] = command_buffer[1];
    pec_buffer[3] = command_buffer[2];
    command_buffer[3] = calculate_pec(pec_buffer, 4);

    printf("Attempting to write new address: 0x%02X\n", new_addr);
    // Write 4 bytes: [Command, Data_LSB, Data_MSB, PEC]
    result = i2c_write_blocking(MLX90614_I2C_PORT, MLX90614_DEFAULT_ADDR, command_buffer, 4, false);
    if (result < 0) {
        printf("Error: Failed to send write command.\n");
        return;
    }
    sleep_ms(10); // Give it time

    printf("✅ Address change command sent. Please POWER CYCLE the sensor now.\n");
}

int main() {
    // Enable serial output
    stdio_init_all();
    sleep_ms(2000); // Wait for serial connection

    while(!stdio_usb_connected()) {
        sleep_ms(100);
    }

    // Initialize I2C
    i2c_init(MLX90614_I2C_PORT, 100 * 1000); // 100kHz
    gpio_set_function(MLX90614_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MLX90614_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MLX90614_I2C_SDA_PIN);
    gpio_pull_up(MLX90614_I2C_SCL_PIN);
    
    sleep_ms(100);

    printf("\n--- Pico I2C Bus Scanner ---\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int row = 0; row < 8; ++row) {
        // Print the row header (00, 10, 20, etc.)
        printf("%02X ", row * 16);

        for (int col = 0; col < 16; ++col) {
            int addr = (row * 16) + col;

            // Skip reserved addresses
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
                continue;
            }

            // Check for a device at this address
            int ret;
            uint8_t dummy; // Not used, just needed for the function call
            
            // i2c_write_blocking returns number of bytes written or an error code (< 0)
            // A successful 0-byte write (ACK received) will return 0.
            ret = i2c_write_blocking(MLX90614_I2C_PORT, addr, &dummy, 1, false);

            if (ret >= 0) {
                printf("%02X ", addr); // Device found, print its address
            } else {
                printf("-- ");       // No device found
            }
        }
        printf("\n"); // Newline at the end of the row
    }

    printf("\nScan complete.\n");

    // <<< SET YOUR DESIRED NEW ADDRESS HERE >>>
    uint8_t new_sensor_address = 0x5B;

    mlx90614_set_address(new_sensor_address);

    // Halt execution
    while (1) {
        tight_loop_contents();
    }
    return 0;
}