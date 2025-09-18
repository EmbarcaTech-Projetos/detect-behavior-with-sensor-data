#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "driver_mpu9250.h"
#include "driver_mlx90614.h"
#include "network.h"
#include "madgwick.h"

// --- Configuration ---
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
const uint8_t MLX_ADDR[5] = {0x5A, 0x5B, 0x5C, 0x5D, 0x5E};
#define SAMPLES 60
#define ACCEL_SENSITIVITY (16384.0f)
#define GYRO_SENSITIVITY (131.0f)
#define GRAVITY_MS2 (9.80665f)

#define FINAL_SAMPLING_RATE_HZ 10 // The effective rate of the data you send (60 samples over 6s = 10 Hz)
#define FAST_SAMPLING_RATE_HZ 500
#define DOWNSAMPLE_FACTOR (FAST_SAMPLING_RATE_HZ / FINAL_SAMPLING_RATE_HZ) // How many fast samples to average
#define FAST_SAMPLE_PERIOD_US (1000000 / FAST_SAMPLING_RATE_HZ) // Period for the fast loop

// --- Data Storage ---
float acc_x_samples[SAMPLES], acc_y_samples[SAMPLES], acc_z_samples[SAMPLES];
float rot_w_samples[SAMPLES], rot_x_samples[SAMPLES], rot_y_samples[SAMPLES], rot_z_samples[SAMPLES];
float thm_samples[5][SAMPLES];

#define JSON_CHUNK_BUFFER_SIZE 256
static char json_chunk[JSON_CHUNK_BUFFER_SIZE];

// --- State Machine ---
typedef enum {
    STATE_IDLE,
    STATE_COLLECTING_DATA,
    STATE_CALCULATING_SIZE,     // New state to accurately calculate payload size
    STATE_START_NETWORK_SEND,
    STATE_WAITING_FOR_CONNECTION,
    STATE_SENDING_DATA,         // Consolidated sending state
    STATE_WAITING_FOR_RESPONSE
} AppState;

// --- Helper Functions ---
void i2c_init_gpio(i2c_inst_t *i2c_port, uint8_t sda_pin, uint8_t scl_pin) {
    i2c_init(i2c_port, 200 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) { sleep_ms(100); }
    printf("--- Pico Data Logger ---\n");

    if (!network_init()) {
        printf("Network initialization failed. Halting.\n");
        while(true);
    }

    // Sensor initializations...
    i2c_init_gpio(MPU9250_I2C_PORT, MPU9250_I2C_SDA_PIN, MPU9250_I2C_SCL_PIN);
    i2c_init_gpio(MLX90614_I2C_PORT, MLX90614_I2C_SDA_PIN, MLX90614_I2C_SCL_PIN);
    mpu9250_init();
    check_mpu9250_identity();

    AppState current_state = STATE_IDLE;
    int sample_count = 0;
    absolute_time_t next_sample_time = nil_time;
    HTTP_REQUEST_STATE_T *current_request = NULL;
    size_t total_content_length = 0; // To store the exact content length

    while (1) {
        cyw43_arch_poll();

        switch (current_state) {
            case STATE_IDLE:
                printf("\nStarting new data collection cycle in 5 seconds...\n");
                sleep_ms(5000);
                sample_count = 0;
                current_state = STATE_COLLECTING_DATA;
                next_sample_time = get_absolute_time();
                break;

            case STATE_COLLECTING_DATA:
                if (sample_count < SAMPLES) {
                    // Accumulators for averaging
                    float acc_x_sum = 0.0f, acc_y_sum = 0.0f, acc_z_sum = 0.0f;
                    float thm_sum[5] = {0.0f};
                    int valid_thm_reads[5] = {0};

                    printf("Downsampling for final sample %d/%d...\n", sample_count + 1, SAMPLES);
                    absolute_time_t next_fast_sample_time = get_absolute_time();

                    // Inner loop to collect data at high frequency
                    for (int i = 0; i < DOWNSAMPLE_FACTOR; i++) {
                        busy_wait_until(next_fast_sample_time);
                        next_fast_sample_time = make_timeout_time_us(FAST_SAMPLE_PERIOD_US);

                        // 1. Read MPU9250
                        int16_t accel[3], gyro[3], mag[3];
                        mpu9250_read_raw_data(accel, gyro, mag);

                        float ax = -(accel[0] / ACCEL_SENSITIVITY) * GRAVITY_MS2;
                        float ay = (accel[1] / ACCEL_SENSITIVITY) * GRAVITY_MS2;
                        float az = -(accel[2] / ACCEL_SENSITIVITY) * GRAVITY_MS2;
                        acc_x_sum += ax;
                        acc_y_sum += ay;
                        acc_z_sum += az;

                        float gx_rad = -(gyro[0] / GYRO_SENSITIVITY) * (M_PI / 180.0f);
                        float gy_rad = (gyro[1] / GYRO_SENSITIVITY) * (M_PI / 180.0f);
                        float gz_rad = -(gyro[2] / GYRO_SENSITIVITY) * (M_PI / 180.0f);
                        
                        // 2. Update orientation filter with every fast sample for accuracy
                        // NOTE: Ensure your Madgwick library is configured with a sample
                        // frequency of FAST_SAMPLING_RATE_HZ (1000 Hz).
                        madgwick_ahrs_update_imu(gx_rad, gy_rad, gz_rad, ax, ay, az, q);
                        
                        // 3. Read ONE MLX90614 sensor per loop to distribute the I2C load
                        uint8_t mlx_idx = i % 5;
                        uint8_t temp_buf[3];
                        if (mlx90614_read_raw_data(MLX_ADDR[mlx_idx], temp_buf) != PICO_ERROR_GENERIC) {
                            uint16_t raw_temp = (uint16_t)temp_buf[1] << 8 | temp_buf[0];
                            if (!(raw_temp & 0x8000)) { // Check for valid reading
                                thm_sum[mlx_idx] += (raw_temp * 0.02) - 273.15;
                                valid_thm_reads[mlx_idx]++;
                            }
                        }
                    } // End of fast sampling loop

                    acc_x_samples[sample_count] = acc_x_sum / DOWNSAMPLE_FACTOR;
                    acc_y_samples[sample_count] = acc_y_sum / DOWNSAMPLE_FACTOR;
                    acc_z_samples[sample_count] = acc_z_sum / DOWNSAMPLE_FACTOR;
                    
                    rot_w_samples[sample_count] = q[0];
                    rot_x_samples[sample_count] = q[1];
                    rot_y_samples[sample_count] = q[2];
                    rot_z_samples[sample_count] = q[3];
                    
                    for(uint8_t i = 0; i < 5; i++) {
                        thm_samples[i][sample_count] = (valid_thm_reads[i] > 0) ? (thm_sum[i] / valid_thm_reads[i]) : 0.0f;
                    }

                    sample_count++;
                }

                if (sample_count >= SAMPLES) {
                    printf("Collection complete.\n");
                    current_state = STATE_CALCULATING_SIZE;
                }
                break;
            
            case STATE_CALCULATING_SIZE:
                printf("Calculating exact payload size...\n");
                total_content_length = 0;
                total_content_length += strlen("{\"sequence\":[");

                for (int i = 0; i < SAMPLES; i++) {
                    // Format the string once to get its true length
                    snprintf(json_chunk, JSON_CHUNK_BUFFER_SIZE,
                             "{\"acc_x\":%.4f,\"acc_y\":%.4f,\"acc_z\":%.4f,"
                             "\"rot_w\":%.4f,\"rot_x\":%.4f,\"rot_y\":%.4f,\"rot_z\":%.4f,"
                             "\"thm_1\":%.2f,\"thm_2\":%.2f,\"thm_3\":%.2f,\"thm_4\":%.2f,\"thm_5\":%.2f}",
                             acc_x_samples[i], acc_y_samples[i], acc_z_samples[i],
                             rot_w_samples[i], rot_x_samples[i], rot_y_samples[i], rot_z_samples[i],
                             thm_samples[0][i], thm_samples[1][i], thm_samples[2][i],
                             thm_samples[3][i], thm_samples[4][i]);
                    if (i > 0) {
                        total_content_length++; // For the comma
                    }
                    total_content_length += strlen(json_chunk);
                }
                total_content_length += strlen("]}");

                printf("Calculated Content-Length: %u\n", total_content_length);
                current_state = STATE_START_NETWORK_SEND;
                break;

            case STATE_START_NETWORK_SEND:
                current_request = network_start_post_request(total_content_length);
                if (current_request) {
                    current_state = STATE_WAITING_FOR_CONNECTION;
                } else {
                    printf("Failed to start network request. Retrying...\n");
                    current_state = STATE_IDLE;
                }
                break;

            case STATE_WAITING_FOR_CONNECTION:
                if (network_is_ready_for_data(current_request)) {
                    printf("Connection ready. Sending JSON data...\n");
                    sample_count = 0; // Reset and use as a progress counter for sending
                    current_state = STATE_SENDING_DATA;
                } else if (network_is_request_complete(current_request)) {
                    printf("Connection failed during setup. Retrying...\n");
                    network_free_request(current_request);
                    current_request = NULL;
                    current_state = STATE_IDLE;
                }
                break;

            case STATE_SENDING_DATA:
                if (network_is_ready_for_data(current_request)) {
                    bool chunk_sent = false;
                    if (sample_count == 0) {
                        chunk_sent = network_write_chunk(current_request, "{\"sequence\":[", strlen("{\"sequence\":["));
                    } else if (sample_count <= SAMPLES) {
                        int sample_idx = sample_count - 1;
                        const char* prefix = (sample_idx == 0) ? "" : ",";
                        
                        // The format string MUST match the one in the calculation state
                        snprintf(json_chunk, JSON_CHUNK_BUFFER_SIZE,
                                 "%s{\"acc_x\":%.4f,\"acc_y\":%.4f,\"acc_z\":%.4f,"
                                 "\"rot_w\":%.4f,\"rot_x\":%.4f,\"rot_y\":%.4f,\"rot_z\":%.4f,"
                                 "\"thm_1\":%.2f,\"thm_2\":%.2f,\"thm_3\":%.2f,\"thm_4\":%.2f,\"thm_5\":%.2f}",
                                 prefix, acc_x_samples[sample_idx], acc_y_samples[sample_idx], acc_z_samples[sample_idx],
                                 rot_w_samples[sample_idx], rot_x_samples[sample_idx], rot_y_samples[sample_idx], rot_z_samples[sample_idx],
                                 thm_samples[0][sample_idx], thm_samples[1][sample_idx], thm_samples[2][sample_idx],
                                 thm_samples[3][sample_idx], thm_samples[4][sample_idx]);
                        
                        int json_l = strlen(json_chunk);
                        printf("%d : %s\n, ", json_l, json_chunk);
                        chunk_sent = network_write_chunk(current_request, json_chunk, strlen(json_chunk));
                    } else { // After all samples are sent
                        chunk_sent = network_write_chunk(current_request, "]}", strlen("]}"));
                    }

                    if (chunk_sent) {
                        if (sample_count == 0) printf("Sent JSON header.\n");
                        else if (sample_count <= SAMPLES) printf("Sent JSON for sample %d/%d.\n", sample_count, SAMPLES);
                        else printf("Sent JSON footer.\n");
                        
                        sample_count++;
                        if (sample_count > SAMPLES + 1) {
                            printf("All data sent. Waiting for server response.\n");
                            current_state = STATE_WAITING_FOR_RESPONSE;
                        }
                    }
                }
                break;

            case STATE_WAITING_FOR_RESPONSE:
                if (network_is_request_complete(current_request)) {
                    printf("Request finished successfully.");
                    network_free_request(current_request);
                    current_request = NULL;
                    current_state = STATE_IDLE;
                }
                break;
        }
    }
    return 0;
}

