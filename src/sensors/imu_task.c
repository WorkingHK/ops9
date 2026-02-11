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

    // Ensure period is at least 1 tick to avoid assertion failure
    if (period == 0) {
        ESP_LOGE(TAG, "Invalid period: SYSTEM_UPDATE_PERIOD_MS=%d results in 0 ticks", SYSTEM_UPDATE_PERIOD_MS);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "IMU task loop started (period=%d ticks)", period);

    int packet_count = 0;
    int read_count = 0;

    while (1) {
        // Read from UART
        int len = imu_driver_read(buffer, sizeof(buffer));

        if (len > 0) {
            read_count++;
            if (read_count % 100 == 0) {
                ESP_LOGI(TAG, "IMU data received: %d bytes (total reads: %d)", len, read_count);
                // Print first 22 bytes as hex for debugging
                ESP_LOG_BUFFER_HEX(TAG, buffer, len > 22 ? 22 : len);
            }

            // Parse all packets in buffer (WIT packets are 11 bytes each)
            for (int i = 0; i < len - 10; i++) {
                if (buffer[i] == 0x55) {
                    bool parsed = imu_driver_parse_packet(&buffer[i], 11,
                                                           &omega_x, &omega_y, &omega_z,
                                                           &accel_x, &accel_y, &accel_z,
                                                           &temperature);

                    if (parsed) {
                        packet_count++;
                        if (packet_count % 100 == 0) {
                            ESP_LOGI(TAG, "IMU parsed: omega_z=%.3f rad/s (packets: %d)", omega_z, packet_count);
                        }

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
                    i += 10;  // Skip to next potential packet
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
