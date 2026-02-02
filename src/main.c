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

static const char *TAG = "OPS9";

// Global shared state
static imu_state_t g_imu_state;
static encoder_state_t g_encoder_state;

// Test sensor tasks
static void test_sensor_tasks(void)
{
    ESP_LOGI(TAG, "Testing sensor tasks...");

    // Initialize hardware drivers
    ESP_LOGI(TAG, "  Initializing hardware drivers...");
    imu_driver_init();
    encoder_driver_init();

    // Start sensor tasks
    ESP_LOGI(TAG, "  Starting IMU task...");
    imu_task_start(&g_imu_state);

    ESP_LOGI(TAG, "  Starting encoder task...");
    encoder_task_start(&g_encoder_state, 0.1f);  // 0.1m wheel diameter

    // Wait for tasks to initialize
    vTaskDelay(pdMS_TO_TICKS(500));

    // Monitor sensor data for 5 seconds
    ESP_LOGI(TAG, "  Monitoring sensor data for 5 seconds...");
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));

        // Read IMU state
        if (xSemaphoreTake(g_imu_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "    IMU: valid=%d, omega=[%.3f, %.3f, %.3f] rad/s, temp=%.1f°C",
                     g_imu_state.valid,
                     g_imu_state.omega_x,
                     g_imu_state.omega_y,
                     g_imu_state.omega_z,
                     g_imu_state.temperature);
            xSemaphoreGive(g_imu_state.mutex);
        }

        // Read encoder state
        if (xSemaphoreTake(g_encoder_state.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG, "    Encoder: valid=%d, velocity=%.3f m/s, count=%lu",
                     g_encoder_state.valid,
                     g_encoder_state.velocity,
                     g_encoder_state.pulse_count);
            xSemaphoreGive(g_encoder_state.mutex);
        }
    }

    ESP_LOGI(TAG, "Sensor tasks tested!");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== OPS9 Positioning System Starting ===");

    nvs_manager_init();

    test_sensor_tasks();

    ESP_LOGI(TAG, "Stage 5: Sensor tasks verified");

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
