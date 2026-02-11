#include "position_task.h"
#include "iakf.h"
#include "attitude.h"
#include "compensation.h"
#include "fusion.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "POS_TASK";

typedef struct {
    imu_state_t *imu_state;
    encoder_state_t *encoder_state;
    position_state_t *position_state;
    calibration_t *calibration;

    // Algorithm state
    iakf_state_t iakf;
    quaternion_t quaternion;
    float x, y;
    uint64_t last_imu_timestamp;
    uint64_t last_encoder_timestamp;
} position_context_t;

static position_context_t g_context;

static void position_task_loop(void *arg)
{
    // Initialize algorithm state
    iakf_init(&g_context.iakf, 0.001f, 0.01f);
    quaternion_init(&g_context.quaternion);
    g_context.x = 0.0f;
    g_context.y = 0.0f;
    g_context.last_imu_timestamp = 0;
    g_context.last_encoder_timestamp = 0;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SYSTEM_UPDATE_PERIOD_MS);
    const float dt = SYSTEM_UPDATE_PERIOD_MS / 1000.0f;

    // Ensure period is at least 1 tick to avoid assertion failure
    if (period == 0) {
        ESP_LOGE(TAG, "Invalid period: SYSTEM_UPDATE_PERIOD_MS=%d results in 0 ticks", SYSTEM_UPDATE_PERIOD_MS);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Position task loop started (200Hz, dt=%.3fs, period=%d ticks)", dt, period);

    while (1) {
        // Copy sensor data
        float omega_x = 0.0f, omega_y = 0.0f, omega_z = 0.0f, temperature = 25.0f;
        float velocity_x = 0.0f, velocity_y = 0.0f;
        bool imu_valid = false, encoder_valid = false;
        uint64_t imu_timestamp = 0, encoder_timestamp = 0;

        // Read IMU state
        if (xSemaphoreTake(g_context.imu_state->mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            omega_x = g_context.imu_state->omega_x;
            omega_y = g_context.imu_state->omega_y;
            omega_z = g_context.imu_state->omega_z;
            temperature = g_context.imu_state->temperature;
            imu_timestamp = g_context.imu_state->timestamp_us;
            imu_valid = g_context.imu_state->valid;
            xSemaphoreGive(g_context.imu_state->mutex);
        }

        // Read encoder state
        if (xSemaphoreTake(g_context.encoder_state->mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            velocity_x = g_context.encoder_state->velocity_x;
            velocity_y = g_context.encoder_state->velocity_y;
            encoder_timestamp = g_context.encoder_state->timestamp_us;
            encoder_valid = g_context.encoder_state->valid;
            xSemaphoreGive(g_context.encoder_state->mutex);
        }

        // Check for sensor timeouts
        uint64_t current_time = esp_timer_get_time();
        bool imu_timeout = (current_time - imu_timestamp) > (IMU_TIMEOUT_MS * 1000);
        bool encoder_timeout = (current_time - encoder_timestamp) > (IMU_TIMEOUT_MS * 1000);

        bool fail_safe = false;
        float confidence = 1.0f;

        if (imu_timeout || !imu_valid) {
            fail_safe = true;
            confidence = 0.5f;
            omega_z = 0.0f;  // Use last known heading
        }

        if (encoder_timeout || !encoder_valid) {
            fail_safe = true;
            confidence = 0.3f;
            velocity_x = 0.0f;
            velocity_y = 0.0f;
        }

        // Apply gyro bias calibration
        if (g_context.calibration->valid) {
            omega_x -= g_context.calibration->gyro_bias[0];
            omega_y -= g_context.calibration->gyro_bias[1];
            omega_z -= g_context.calibration->gyro_bias[2];
        }

        // Algorithm pipeline
        // 1. IAKF
        float omega_z_filtered = iakf_update(&g_context.iakf, omega_z);

        // 2. Temperature compensation
        float omega_z_temp = compensation_temperature_drift(omega_z_filtered, temperature,
                                                             g_context.calibration);

        // 3. Dynamic state compensation
        bool is_moving = (fabsf(velocity_x) > 0.01f) || (fabsf(velocity_y) > 0.01f);
        float velocity_magnitude = sqrtf(velocity_x * velocity_x + velocity_y * velocity_y);
        float omega_z_final = compensation_dynamic_state(omega_z_temp, velocity_magnitude, is_moving);

        // 4. Attitude solving
        quaternion_update_rk2(&g_context.quaternion, omega_x, omega_y, omega_z_final, dt);
        float heading = quaternion_to_heading(&g_context.quaternion);

        // 5. Decouple rotation from encoder measurements (chassis frame)
        float V_x_chassis = velocity_x + ENCODER_X_ROTATION_COUPLING * omega_z_final;
        float V_y_chassis = velocity_y + ENCODER_Y_ROTATION_COUPLING * omega_z_final;

        // 6. Transform chassis velocities to global frame
        float cos_heading = cosf(heading);
        float sin_heading = sinf(heading);
        float V_x_global = V_x_chassis * cos_heading - V_y_chassis * sin_heading;
        float V_y_global = V_x_chassis * sin_heading + V_y_chassis * cos_heading;

        // 7. Integrate position
        g_context.x += V_x_global * dt;
        g_context.y += V_y_global * dt;

        // Update position state
        if (xSemaphoreTake(g_context.position_state->mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_context.position_state->x = g_context.x;
            g_context.position_state->y = g_context.y;
            g_context.position_state->heading = heading;
            g_context.position_state->confidence = confidence;
            g_context.position_state->fail_safe_mode = fail_safe;
            g_context.position_state->timestamp_us = current_time;
            xSemaphoreGive(g_context.position_state->mutex);
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

esp_err_t position_task_start(imu_state_t *imu_state,
                               encoder_state_t *encoder_state,
                               position_state_t *position_state,
                               calibration_t *calibration)
{
    g_context.imu_state = imu_state;
    g_context.encoder_state = encoder_state;
    g_context.position_state = position_state;
    g_context.calibration = calibration;

    position_state->mutex = xSemaphoreCreateMutex();
    position_state->fail_safe_mode = false;
    position_state->confidence = 0.0f;

    xTaskCreatePinnedToCore(position_task_loop, "position_task", 8192, NULL, 4, NULL, 1);
    ESP_LOGI(TAG, "Position task started on core 1");
    return ESP_OK;
}
