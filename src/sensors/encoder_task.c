#include "encoder_task.h"
#include "hal/encoder_driver.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "ENC_TASK";
static encoder_state_t *g_encoder_state = NULL;
static float g_wheel_diameter = 0.1f;  // meters

static void encoder_task_loop(void *arg)
{
    int32_t last_count = 0;
    uint64_t last_time_us = esp_timer_get_time();

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SYSTEM_UPDATE_PERIOD_MS);

    ESP_LOGI(TAG, "Encoder task loop started");

    while (1) {
        int32_t current_count = encoder_driver_get_count();
        uint64_t current_time_us = esp_timer_get_time();

        int32_t delta_count = current_count - last_count;
        uint64_t delta_time_us = current_time_us - last_time_us;

        if (delta_time_us > 0) {
            // Calculate velocity
            float distance_per_pulse = (M_PI * g_wheel_diameter) / ENCODER_PPR;
            float distance = delta_count * distance_per_pulse;
            float velocity = distance / (delta_time_us / 1000000.0f);

            // Update shared state
            if (xSemaphoreTake(g_encoder_state->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_encoder_state->velocity = velocity;
                g_encoder_state->pulse_count = current_count;
                g_encoder_state->timestamp_us = current_time_us;
                g_encoder_state->valid = true;
                xSemaphoreGive(g_encoder_state->mutex);
            }
        }

        last_count = current_count;
        last_time_us = current_time_us;

        vTaskDelayUntil(&last_wake_time, period);
    }
}

esp_err_t encoder_task_start(encoder_state_t *shared_state, float wheel_diameter)
{
    g_encoder_state = shared_state;
    g_wheel_diameter = wheel_diameter;
    g_encoder_state->mutex = xSemaphoreCreateMutex();
    g_encoder_state->valid = false;

    xTaskCreatePinnedToCore(encoder_task_loop, "encoder_task", 2048, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Encoder task started on core 0 (wheel_diameter=%.3fm)", wheel_diameter);
    return ESP_OK;
}
