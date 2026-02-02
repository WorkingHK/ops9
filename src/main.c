#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"
#include "types.h"
#include "hal/imu_driver.h"
#include "hal/encoder_driver.h"
#include "hal/nvs_manager.h"
#include "sensors/imu_task.h"
#include "sensors/encoder_task.h"
#include "processing/position_task.h"
#include "output/uart_output.h"

static const char *TAG = "MAIN";

// Global shared state
static imu_state_t g_imu_state;
static encoder_state_t g_encoder_state;
static position_state_t g_position_state;
static calibration_t g_calibration;

void app_main(void)
{
    ESP_LOGI(TAG, "=== OPS9 Positioning System Starting ===");

    // Initialize NVS
    ESP_ERROR_CHECK(nvs_manager_init());

    // Load calibration
    esp_err_t cal_err = nvs_manager_load_calibration(&g_calibration);
    if (cal_err != ESP_OK) {
        ESP_LOGW(TAG, "No calibration found, using defaults");
        g_calibration.valid = false;
        // Set default values
        for (int i = 0; i < 3; i++) {
            g_calibration.gyro_bias[i] = 0.0f;
            g_calibration.temp_drift_coeffs[i] = 0.0f;
        }
        g_calibration.wheel_angle = 0.0f;
    } else {
        ESP_LOGI(TAG, "Calibration loaded successfully");
    }

    // Initialize hardware drivers
    ESP_ERROR_CHECK(imu_driver_init());
    ESP_ERROR_CHECK(encoder_driver_init());

    // Start sensor tasks
    ESP_ERROR_CHECK(imu_task_start(&g_imu_state));
    ESP_ERROR_CHECK(encoder_task_start(&g_encoder_state, 0.1f));  // 0.1m wheel diameter

    // Wait for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Start processing task
    ESP_ERROR_CHECK(position_task_start(&g_imu_state, &g_encoder_state,
                                        &g_position_state, &g_calibration));

    // Start output task
    ESP_ERROR_CHECK(uart_output_start(&g_position_state));

    ESP_LOGI(TAG, "=== All tasks started, system running ===");

    // Main loop - monitor system health
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Read and log position
        if (xSemaphoreTake(g_position_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "Position: x=%.3f y=%.3f heading=%.2f° conf=%.2f fail=%d",
                     g_position_state.x,
                     g_position_state.y,
                     g_position_state.heading * 180.0f / 3.14159f,
                     g_position_state.confidence,
                     g_position_state.fail_safe_mode);
            xSemaphoreGive(g_position_state.mutex);
        }
    }
}
