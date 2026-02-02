#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "types.h"
#include "hal/nvs_manager.h"
#include "hal/imu_driver.h"
#include "hal/encoder_driver.h"

static const char *TAG = "OPS9";

// Test HAL drivers
static void test_hal_drivers(void)
{
    ESP_LOGI(TAG, "Testing HAL drivers...");

    // Test NVS
    ESP_LOGI(TAG, "  Initializing NVS...");
    esp_err_t err = nvs_manager_init();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "  NVS: OK");

        // Test calibration save/load
        calibration_t cal = {
            .gyro_bias = {0.001f, -0.002f, 0.003f},
            .temp_drift_coeffs = {0.0f, 0.0f, 0.0f},
            .wheel_angle = 0.0f,
            .valid = true
        };

        nvs_manager_save_calibration(&cal);

        calibration_t loaded_cal;
        err = nvs_manager_load_calibration(&loaded_cal);
        if (err == ESP_OK && loaded_cal.valid) {
            ESP_LOGI(TAG, "  Calibration save/load: OK (bias=[%.6f, %.6f, %.6f])",
                     loaded_cal.gyro_bias[0], loaded_cal.gyro_bias[1], loaded_cal.gyro_bias[2]);
        }
    }

    // Test IMU driver
    ESP_LOGI(TAG, "  Initializing IMU driver...");
    err = imu_driver_init();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "  IMU driver: OK");

        // Test packet parsing with dummy data
        uint8_t test_packet[] = {0x55, 0x52, 0x00, 0x10, 0x00, 0x20, 0x00, 0x30, 0x00, 0x00, 0x00};
        float wx, wy, wz, ax, ay, az, temp;
        bool parsed = imu_driver_parse_packet(test_packet, sizeof(test_packet),
                                               &wx, &wy, &wz, &ax, &ay, &az, &temp);
        if (parsed) {
            ESP_LOGI(TAG, "  IMU packet parsing: OK (omega=[%.3f, %.3f, %.3f] rad/s)",
                     wx, wy, wz);
        }
    }

    // Test encoder driver
    ESP_LOGI(TAG, "  Initializing encoder driver...");
    err = encoder_driver_init();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "  Encoder driver: OK");

        int32_t count = encoder_driver_get_count();
        ESP_LOGI(TAG, "  Encoder count: %ld", count);

        encoder_driver_reset();
        count = encoder_driver_get_count();
        ESP_LOGI(TAG, "  Encoder after reset: %ld", count);
    }

    ESP_LOGI(TAG, "All HAL drivers tested!");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== OPS9 Positioning System Starting ===");

    test_hal_drivers();

    ESP_LOGI(TAG, "Stage 3: HAL drivers verified");

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
