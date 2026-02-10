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
static float g_wheel_x_diameter = 0.1f;  // meters
static float g_wheel_y_diameter = 0.1f;  // meters

static void encoder_task_loop(void *arg)
{
    int32_t last_count_x = 0;
    int32_t last_count_y = 0;
    uint64_t last_time_us = esp_timer_get_time();

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SYSTEM_UPDATE_PERIOD_MS);

    ESP_LOGI(TAG, "Encoder task loop started (dual X/Y)");

    while (1) {
        int32_t current_count_x = encoder_driver_get_count(ENCODER_X);
        int32_t current_count_y = encoder_driver_get_count(ENCODER_Y);
        uint64_t current_time_us = esp_timer_get_time();

        int32_t delta_count_x = current_count_x - last_count_x;
        int32_t delta_count_y = current_count_y - last_count_y;
        uint64_t delta_time_us = current_time_us - last_time_us;

        if (delta_time_us > 0) {
            // Calculate X velocity
            float distance_per_pulse_x = (M_PI * g_wheel_x_diameter) / ENCODER_X_PPR;
            float distance_x = delta_count_x * distance_per_pulse_x;
            float velocity_x = distance_x / (delta_time_us / 1000000.0f);

            // Calculate Y velocity
            float distance_per_pulse_y = (M_PI * g_wheel_y_diameter) / ENCODER_Y_PPR;
            float distance_y = delta_count_y * distance_per_pulse_y;
            float velocity_y = distance_y / (delta_time_us / 1000000.0f);

            // Update shared state
            if (xSemaphoreTake(g_encoder_state->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_encoder_state->velocity_x = velocity_x;
                g_encoder_state->velocity_y = velocity_y;
                g_encoder_state->pulse_count_x = current_count_x;
                g_encoder_state->pulse_count_y = current_count_y;
                g_encoder_state->timestamp_us = current_time_us;
                g_encoder_state->valid = true;
                xSemaphoreGive(g_encoder_state->mutex);
            }
        }

        last_count_x = current_count_x;
        last_count_y = current_count_y;
        last_time_us = current_time_us;

        vTaskDelayUntil(&last_wake_time, period);
    }
}

esp_err_t encoder_task_start(encoder_state_t *shared_state,
                              float wheel_x_diameter,
                              float wheel_y_diameter)
{
    g_encoder_state = shared_state;
    g_wheel_x_diameter = wheel_x_diameter;
    g_wheel_y_diameter = wheel_y_diameter;
    g_encoder_state->mutex = xSemaphoreCreateMutex();
    g_encoder_state->valid = false;

    xTaskCreatePinnedToCore(encoder_task_loop, "encoder_task", 2048, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Encoder task started on core 0 (X_diam=%.3fm, Y_diam=%.3fm)",
             wheel_x_diameter, wheel_y_diameter);
    return ESP_OK;
}
