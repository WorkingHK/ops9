#include "command_parser.h"
#include "hal/nvs_manager.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "CMD_PARSER";
static imu_state_t *g_imu_state = NULL;
static calibration_t *g_calibration = NULL;

static void cmd_cal_gyro(const char *args)
{
    ESP_LOGI(TAG, "Starting gyro bias calibration (30s)...");
    ESP_LOGI(TAG, "Keep the system STATIONARY!");

    if (!g_imu_state || !g_calibration) {
        ESP_LOGE(TAG, "Context not set");
        return;
    }

    // Accumulate samples
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 0;

    for (int i = 0; i < GYRO_BIAS_CAL_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(SYSTEM_UPDATE_PERIOD_MS));

        if (xSemaphoreTake(g_imu_state->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (g_imu_state->valid) {
                sum_x += g_imu_state->omega_x;
                sum_y += g_imu_state->omega_y;
                sum_z += g_imu_state->omega_z;
                samples++;
            }
            xSemaphoreGive(g_imu_state->mutex);
        }

        if (i % 200 == 0) {
            ESP_LOGI(TAG, "Progress: %d%%", (i * 100) / GYRO_BIAS_CAL_SAMPLES);
        }
    }

    if (samples > 0) {
        g_calibration->gyro_bias[0] = sum_x / samples;
        g_calibration->gyro_bias[1] = sum_y / samples;
        g_calibration->gyro_bias[2] = sum_z / samples;
        g_calibration->valid = true;

        nvs_manager_save_calibration(g_calibration);

        ESP_LOGI(TAG, "Gyro bias calibration complete!");
        ESP_LOGI(TAG, "  Bias X: %.6f rad/s", g_calibration->gyro_bias[0]);
        ESP_LOGI(TAG, "  Bias Y: %.6f rad/s", g_calibration->gyro_bias[1]);
        ESP_LOGI(TAG, "  Bias Z: %.6f rad/s", g_calibration->gyro_bias[2]);
        ESP_LOGI(TAG, "  Samples: %d", samples);
    } else {
        ESP_LOGE(TAG, "No valid samples collected");
    }
}

static void cmd_cal_status(const char *args)
{
    if (!g_calibration) {
        ESP_LOGE(TAG, "Context not set");
        return;
    }

    ESP_LOGI(TAG, "=== Calibration Status ===");
    ESP_LOGI(TAG, "  Valid: %s", g_calibration->valid ? "YES" : "NO");
    ESP_LOGI(TAG, "  Gyro bias:");
    ESP_LOGI(TAG, "    X: %.6f rad/s", g_calibration->gyro_bias[0]);
    ESP_LOGI(TAG, "    Y: %.6f rad/s", g_calibration->gyro_bias[1]);
    ESP_LOGI(TAG, "    Z: %.6f rad/s", g_calibration->gyro_bias[2]);
    ESP_LOGI(TAG, "  Temp drift coeffs:");
    ESP_LOGI(TAG, "    c0: %.6f", g_calibration->temp_drift_coeffs[0]);
    ESP_LOGI(TAG, "    c1: %.6f", g_calibration->temp_drift_coeffs[1]);
    ESP_LOGI(TAG, "    c2: %.6f", g_calibration->temp_drift_coeffs[2]);
    ESP_LOGI(TAG, "  Wheel angle: %.6f rad (%.2f deg)",
             g_calibration->wheel_angle,
             g_calibration->wheel_angle * 180.0f / 3.14159f);
}

static void cmd_cal_reset(const char *args)
{
    ESP_LOGI(TAG, "Resetting calibration...");
    nvs_manager_clear_calibration();

    if (g_calibration) {
        g_calibration->valid = false;
        memset(g_calibration, 0, sizeof(calibration_t));
    }

    ESP_LOGI(TAG, "Calibration reset complete");
}

static void cmd_cal_wheel(const char *args)
{
    if (!g_calibration) {
        ESP_LOGE(TAG, "Context not set");
        return;
    }

    float angle = atof(args);
    g_calibration->wheel_angle = angle;
    g_calibration->valid = true;

    nvs_manager_save_calibration(g_calibration);
    ESP_LOGI(TAG, "Wheel angle set to %.6f rad (%.2f deg)",
             angle, angle * 180.0f / 3.14159f);
}

static void cmd_help(const char *args)
{
    ESP_LOGI(TAG, "=== Available Commands ===");
    ESP_LOGI(TAG, "  CAL_GYRO        - Run gyro bias calibration (30s)");
    ESP_LOGI(TAG, "  CAL_STATUS      - Show calibration status");
    ESP_LOGI(TAG, "  CAL_RESET       - Clear calibration");
    ESP_LOGI(TAG, "  CAL_WHEEL <rad> - Set wheel angle (radians)");
    ESP_LOGI(TAG, "  HELP            - Show this help");
}

void command_parser_init(void)
{
    ESP_LOGI(TAG, "Command parser initialized");
    ESP_LOGI(TAG, "Type 'HELP' for available commands");
}

void command_parser_process(const char *cmd_string)
{
    if (strncmp(cmd_string, "CAL_GYRO", 8) == 0) {
        cmd_cal_gyro(NULL);
    } else if (strncmp(cmd_string, "CAL_STATUS", 10) == 0) {
        cmd_cal_status(NULL);
    } else if (strncmp(cmd_string, "CAL_RESET", 9) == 0) {
        cmd_cal_reset(NULL);
    } else if (strncmp(cmd_string, "CAL_WHEEL", 9) == 0) {
        cmd_cal_wheel(cmd_string + 10);
    } else if (strncmp(cmd_string, "HELP", 4) == 0) {
        cmd_help(NULL);
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd_string);
        ESP_LOGI(TAG, "Type 'HELP' for available commands");
    }
}

void command_parser_set_context(imu_state_t *imu_state, calibration_t *calibration)
{
    g_imu_state = imu_state;
    g_calibration = calibration;
    ESP_LOGI(TAG, "Context set");
}
