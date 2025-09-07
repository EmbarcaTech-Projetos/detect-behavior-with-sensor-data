#include <stdio.h>
#include <string.h>
#include <math.h> // Added for M_PI
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "driver_mpu9250.h"
#include "driver_mlx90614.h"
#include "network.h"
#include "madgwick.h"

// Correctly initialized identity quaternion (w, x, y, z)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// --- Configuration ---
#define SAMPLING_RATE 10 // Hz
#define SAMPLES 60       // Number of samples to collect before sending

// --- MPU-9250 Conversion Constants ---
// Sensitivity based on default settings in mpu9250_init()
// Accel: ±2g -> (2^15 - 1) / 2 = 16384 LSB/g
// Gyro: ±250°/s -> (2^15 - 1) / 250 = 131 LSB/(°/s)
#define ACCEL_SENSITIVITY (16384.0f)
#define GYRO_SENSITIVITY (131.0f)
#define GRAVITY_MS2 (9.80665f)

// --- Data Storage ---
// Buffers to hold the collected samples before sending
float acc_x_samples[SAMPLES];
float acc_y_samples[SAMPLES];
float acc_z_samples[SAMPLES];

// Buffers for quaternion orientation data
float rot_w_samples[SAMPLES];
float rot_x_samples[SAMPLES];
float rot_y_samples[SAMPLES];
float rot_z_samples[SAMPLES];

float thm_1_samples[SAMPLES];

// --- Network & JSON ---
#define JSON_BUFFER_SIZE 8192 // Allocate a larger buffer for the batch JSON
static char json_payload[JSON_BUFFER_SIZE];
static HTTP_REQUEST_STATE_T *current_request; // Assuming this is defined in network.h

/**
 * @brief Formats the collected sensor data into a single JSON object.
 * The format is an object containing an array of sample objects.
 * @param buffer The character buffer to write the JSON string into.
 * @param buffer_size The size of the buffer.
 */
void format_json_request(char *buffer, size_t buffer_size) {
    char *ptr = buffer;
    int remaining_size = buffer_size;
    int written;

    // Start JSON object and samples array
    written = snprintf(ptr, remaining_size, "{\"samples\":[");
    ptr += written; remaining_size -= written;

    // Loop through each sample to create its JSON object
    for (int i = 0; i < SAMPLES; i++) {
        // Start of sample object
        written = snprintf(ptr, remaining_size, "{");
        ptr += written; remaining_size -= written;

        // Append all sensor data for the current sample in a single call
        written = snprintf(ptr, remaining_size,
                           "\"acc_x\":%.4f,"
                           "\"acc_y\":%.4f,"
                           "\"acc_z\":%.4f,"
                           "\"rot_w\":%.4f,"
                           "\"rot_x\":%.4f,"
                           "\"rot_y\":%.4f,"
                           "\"rot_z\":%.4f,"
                           "\"thm_1\":%.2f,"
                           "\"thm_2\":%.2f,"
                           "\"thm_3\":%.2f,"
                           "\"thm_4\":%.2f,"
                           "\"thm_5\":%.2f",
                           acc_x_samples[i], acc_y_samples[i], acc_z_samples[i],
                           rot_w_samples[i], rot_x_samples[i], rot_y_samples[i], rot_z_samples[i],
                           thm_1_samples[i], thm_1_samples[i], thm_1_samples[i], // Re-use thm_1 value
                           thm_1_samples[i], thm_1_samples[i]);
        ptr += written; remaining_size -= written;

        // End of sample object
        written = snprintf(ptr, remaining_size, "}");
        ptr += written; remaining_size -= written;

        // Add a comma between objects, but not after the last one
        if (i < SAMPLES - 1) {
            written = snprintf(ptr, remaining_size, ",");
            ptr += written; remaining_size -= written;
        }
    }

    // End of samples array and main JSON object
    snprintf(ptr, remaining_size, "]}");
}

int main() {
    stdio_init_all();

    // Wait for the serial port to be opened by the host
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("--- Pico MPU-9250 & MLX90614 Data Logger ---\n");

    // Initialize the network (only once)
    if (!network_init()) {
        printf("Network initialization failed. Halting.\n");
        while(true);
    }
    
    // Initialize I2C for MPU9250
    i2c_init(MPU9250_I2C_PORT, 100 * 1000);
    gpio_set_function(MPU9250_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU9250_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU9250_I2C_SDA_PIN);
    gpio_pull_up(MPU9250_I2C_SCL_PIN);

    // Initialize I2C for MLX90614
    i2c_init(MLX90614_I2C_PORT, 100 * 1000); 
    gpio_set_function(MLX90614_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MLX90614_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MLX90614_I2C_SDA_PIN);
    gpio_pull_up(MLX90614_I2C_SCL_PIN);
    
    printf("Attempting to initialize MPU-9250...\n");
    mpu9250_init();
    check_mpu9250_identity();

    // Buffers for raw sensor data
    int16_t accelerometer[3], gyroscope[3], magnetometer[3];
    uint8_t temp_read_buffer[3];
    int sample_count = 0;
    const uint32_t delay_ms = 1000 / SAMPLING_RATE;

    // --- Main Loop ---
    while (1) {
        printf("--- Starting new data collection cycle (%d samples at %d Hz) ---\n", SAMPLES, SAMPLING_RATE);
        
        // --- 1. Data Collection Phase ---
        for (sample_count = 0; sample_count < SAMPLES; sample_count++) {
            cyw43_arch_poll(); // Keep network stack running during collection

            // Read MPU-9250
            mpu9250_read_raw_data(accelerometer, gyroscope, magnetometer);
            
            // Read MLX90614
            int mlx_result = mlx90614_read_raw_data(temp_read_buffer);

            // --- Convert and Store Data ---

            // Accelerometer: Raw -> m/s^2
            acc_x_samples[sample_count] = (accelerometer[0] / ACCEL_SENSITIVITY) * GRAVITY_MS2;
            acc_y_samples[sample_count] = (accelerometer[1] / ACCEL_SENSITIVITY) * GRAVITY_MS2;
            acc_z_samples[sample_count] = (accelerometer[2] / ACCEL_SENSITIVITY) * GRAVITY_MS2;
            
            // Gyroscope: Raw -> radians/sec
            float gx_rad = (gyroscope[0] / GYRO_SENSITIVITY) * (M_PI / 180.0f);
            float gy_rad = (gyroscope[1] / GYRO_SENSITIVITY) * (M_PI / 180.0f);
            float gz_rad = (gyroscope[2] / GYRO_SENSITIVITY) * (M_PI / 180.0f);

            // Update the AHRS algorithm with the new sensor values
            madgwick_ahrs_update_imu(
                gx_rad, gy_rad, gz_rad,
                acc_x_samples[sample_count], acc_y_samples[sample_count], acc_z_samples[sample_count],
                q
            );

            // Store the resulting orientation quaternion
            rot_w_samples[sample_count] = q[0];
            rot_x_samples[sample_count] = q[1];
            rot_y_samples[sample_count] = q[2];
            rot_z_samples[sample_count] = q[3];

            // Temperature: Raw -> °C
            if (mlx_result != PICO_ERROR_GENERIC) {
                uint16_t raw_temp = (uint16_t)temp_read_buffer[1] << 8 | temp_read_buffer[0];
                if (!(raw_temp & 0x8000)) { // Check for error flag
                    thm_1_samples[sample_count] = (raw_temp * 0.02) - 273.15;
                } else {
                    thm_1_samples[sample_count] = 0.0f; // Store 0 on error
                }
            } else {
                thm_1_samples[sample_count] = 0.0f; // Store 0 on error
            }

            printf("Sample %d/%d collected.\n", sample_count + 1, SAMPLES);
            sleep_ms(delay_ms);
        }

        printf("--- Collection complete. Preparing to send data. ---\n");

        // --- 2. Network Sending Phase ---
        if (current_request == NULL || current_request->complete) {
            if (current_request != NULL) {
                free(current_request);
                current_request = NULL;
            }

            // Format the JSON payload with all collected samples
            format_json_request(json_payload, JSON_BUFFER_SIZE);
            
            // DEBUG: Print the JSON payload before sending
            printf("JSON Payload: %s\n", json_payload);

            printf("Sending HTTP POST request...\n");
            current_request = network_send_post_request(json_payload);
            if (current_request == NULL) {
                printf("Failed to start HTTP POST request.\n");
            } else {
                printf("Request initiated. Will proceed to next collection cycle.\n");
            }
        } else {
            // If a request is still in progress, we drop the current batch of data
            // and start a new collection. This prevents data from piling up if the
            // network is slow.
            printf("Network busy, previous request not complete. Dropping current data batch.\n");
        }
        
        // A small delay to allow network functions to process before restarting the loop
        sleep_ms(100);
    }

    return 0;
}