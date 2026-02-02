#include "imu_task.h"
#include "hal/imu_driver.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IMU_TASK";
static imu_state_t *g_imu_state = NULL;

static void imu_task_loop(void *arg)
{
    uint8_t buffer[256];
    float omega_x = 0, omega_y = 0, omega_z = 0;
    float accel_x = 0, accel_y = 0, accel_z = 0;
    float temperature = 25.0f;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SYSTEM_UPDATE_PERIOD_MS);

    ESP_LOGI(TAG, "IMU task loop started");

    while (1) {
        // Read from UART
        int len = imu_driver_read(buffer, sizeof(buffer));

        if (len > 0) {
            // Parse WIT protocol packets
            bool parsed = imu_driver_parse_packet(buffer, len,
                                                   &omega_x, &omega_y, &omega_z,
                                                   &accel_x, &accel_y, &accel_z,
                                                   &temperature);

            if (parsed) {
                // Update shared state
                if (xSemaphoreTake(g_imu_state->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_imu_state->omega_x = omega_x;
                    g_imu_state->omega_y = omega_y;
                    g_imu_state->omega_z = omega_z;
                    g_imu_state->accel_x = accel_x;
                    g_imu_state->accel_y = accel_y;
                    g_imu_state->accel_z = accel_z;
                    g_imu_state->temperature = temperature;
                    g_imu_state->timestamp_us = esp_timer_get_time();
                    g_imu_state->valid = true;
                    xSemaphoreGive(g_imu_state->mutex);
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

esp_err_t imu_task_start(imu_state_t *shared_state)
{
    g_imu_state = shared_state;
    g_imu_state->mutex = xSemaphoreCreateMutex();
    g_imu_state->valid = false;

    xTaskCreatePinnedToCore(imu_task_loop, "imu_task", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "IMU task started on core 0");
    return ESP_OK;
}
