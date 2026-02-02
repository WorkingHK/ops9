#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "types.h"
#include "hal/nvs_manager.h"
#include "hal/imu_driver.h"
#include "hal/encoder_driver.h"
#include "sensors/imu_task.h"
#include "sensors/encoder_task.h"
#include "processing/position_task.h"

static const char *TAG = "OPS9";

// Global shared state
static imu_state_t g_imu_state;
static encoder_state_t g_encoder_state;
static position_state_t g_position_state;
static calibration_t g_calibration;

// Test position processing task
static void test_position_task(void)
{
    ESP_LOGI(TAG, "Testing position processing task...");

    // Load calibration
    esp_err_t cal_err = nvs_manager_load_calibration(&g_calibration);
    if (cal_err != ESP_OK) {
        ESP_LOGW(TAG, "  No calibration found, using defaults");
        g_calibration.valid = false;
        for (int i = 0; i < 3; i++) {
            g_calibration.gyro_bias[i] = 0.0f;
            g_calibration.temp_drift_coeffs[i] = 0.0f;
        }
        g_calibration.wheel_angle = 0.0f;
    }

    // Initialize hardware drivers
    ESP_LOGI(TAG, "  Initializing hardware drivers...");
    imu_driver_init();
    encoder_driver_init();

    // Start sensor tasks
    ESP_LOGI(TAG, "  Starting sensor tasks...");
    imu_task_start(&g_imu_state);
    encoder_task_start(&g_encoder_state, 0.1f);  // 0.1m wheel diameter

    // Wait for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Start processing task
    ESP_LOGI(TAG, "  Starting position processing task...");
    position_task_start(&g_imu_state, &g_encoder_state, &g_position_state, &g_calibration);

    // Monitor position for 10 seconds
    ESP_LOGI(TAG, "  Monitoring position for 10 seconds...");
    for (int i = 0; i < 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));

        // Read position state
        if (xSemaphoreTake(g_position_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "    Position: x=%.3f y=%.3f heading=%.2f° conf=%.2f fail=%d",
                     g_position_state.x,
                     g_position_state.y,
                     g_position_state.heading * 180.0f / 3.14159f,
                     g_position_state.confidence,
                     g_position_state.fail_safe_mode);
            xSemaphoreGive(g_position_state.mutex);
        }
    }

    ESP_LOGI(TAG, "Position processing task tested!");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== OPS9 Positioning System Starting ===");

    nvs_manager_init();

    test_position_task();

    ESP_LOGI(TAG, "Stage 6: Position processing task verified");

    // Continue monitoring
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Read and log position
        if (xSemaphoreTake(g_position_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "Position: x=%.3f y=%.3f heading=%.2f° conf=%.2f",
                     g_position_state.x,
                     g_position_state.y,
                     g_position_state.heading * 180.0f / 3.14159f,
                     g_position_state.confidence);
            xSemaphoreGive(g_position_state.mutex);
        }
    }
}
