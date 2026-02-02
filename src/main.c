#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "types.h"

static const char *TAG = "OPS9";

// Test function to verify data structures compile correctly
static void test_data_structures(void)
{
    ESP_LOGI(TAG, "Testing data structures...");

    // Test IMU state
    imu_state_t imu_state = {
        .omega_x = 0.0f,
        .omega_y = 0.0f,
        .omega_z = 0.0f,
        .valid = false,
        .mutex = NULL
    };
    ESP_LOGI(TAG, "  IMU state size: %d bytes", sizeof(imu_state_t));

    // Test encoder state
    encoder_state_t encoder_state = {
        .velocity = 0.0f,
        .pulse_count = 0,
        .valid = false,
        .mutex = NULL
    };
    ESP_LOGI(TAG, "  Encoder state size: %d bytes", sizeof(encoder_state_t));

    // Test position state
    position_state_t position_state = {
        .x = 0.0f,
        .y = 0.0f,
        .heading = 0.0f,
        .confidence = 0.0f,
        .fail_safe_mode = false,
        .mutex = NULL
    };
    ESP_LOGI(TAG, "  Position state size: %d bytes", sizeof(position_state_t));

    // Test calibration
    calibration_t calibration = {
        .gyro_bias = {0.0f, 0.0f, 0.0f},
        .valid = false
    };
    ESP_LOGI(TAG, "  Calibration size: %d bytes", sizeof(calibration_t));

    // Test quaternion
    quaternion_t quat = {1.0f, 0.0f, 0.0f, 0.0f};
    ESP_LOGI(TAG, "  Quaternion size: %d bytes", sizeof(quaternion_t));

    ESP_LOGI(TAG, "All data structures OK!");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== OPS9 Positioning System Starting ===");

    test_data_structures();

    ESP_LOGI(TAG, "Stage 2: Data structures verified");

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
