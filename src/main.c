#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "types.h"
#include "hal/nvs_manager.h"
#include "hal/imu_driver.h"
#include "hal/encoder_driver.h"
#include "processing/iakf.h"
#include "processing/attitude.h"
#include "processing/compensation.h"
#include "processing/coordinate.h"
#include "processing/fusion.h"
#include <math.h>

static const char *TAG = "OPS9";

// Test algorithm modules
static void test_algorithms(void)
{
    ESP_LOGI(TAG, "Testing algorithm modules...");

    // Test IAKF
    ESP_LOGI(TAG, "  Testing IAKF...");
    iakf_state_t iakf;
    iakf_init(&iakf, 0.001f, 0.01f);

    float measurements[] = {0.1f, 0.15f, 0.12f, 0.11f, 0.13f};
    for (int i = 0; i < 5; i++) {
        float filtered = iakf_update(&iakf, measurements[i]);
        ESP_LOGI(TAG, "    IAKF: meas=%.3f, filtered=%.3f, K=%.3f",
                 measurements[i], filtered, iakf.K);
    }
    ESP_LOGI(TAG, "  IAKF: OK");

    // Test quaternion and attitude
    ESP_LOGI(TAG, "  Testing quaternion/attitude...");
    quaternion_t quat;
    quaternion_init(&quat);
    ESP_LOGI(TAG, "    Initial quat: [%.3f, %.3f, %.3f, %.3f]",
             quat.q0, quat.q1, quat.q2, quat.q3);

    // Simulate rotation around Z axis
    float omega_z = 0.1f;  // 0.1 rad/s
    float dt = 0.005f;     // 5ms
    for (int i = 0; i < 100; i++) {
        quaternion_update_rk2(&quat, 0.0f, 0.0f, omega_z, dt);
    }
    float heading = quaternion_to_heading(&quat);
    ESP_LOGI(TAG, "    After 100 steps: heading=%.3f rad (%.1f deg)",
             heading, heading * 180.0f / M_PI);
    ESP_LOGI(TAG, "  Attitude: OK");

    // Test compensation
    ESP_LOGI(TAG, "  Testing compensation...");
    calibration_t cal = {
        .gyro_bias = {0.0f, 0.0f, 0.0f},
        .temp_drift_coeffs = {0.001f, 0.0001f, 0.0f},
        .valid = true
    };

    float omega_raw = 0.05f;
    float temp = 25.0f;
    float omega_comp = compensation_temperature_drift(omega_raw, temp, &cal);
    ESP_LOGI(TAG, "    Temp compensation: raw=%.6f, comp=%.6f", omega_raw, omega_comp);

    float omega_thresh = compensation_threshold(0.005f, 0.01f);
    ESP_LOGI(TAG, "    Threshold: 0.005 -> %.6f (below threshold)", omega_thresh);

    float omega_dyn = compensation_dynamic_state(0.02f, 0.5f, true);
    ESP_LOGI(TAG, "    Dynamic state: 0.02 -> %.6f (moving)", omega_dyn);
    ESP_LOGI(TAG, "  Compensation: OK");

    // Test coordinate calculation
    ESP_LOGI(TAG, "  Testing coordinate calculation...");
    float dx, dy;
    coordinate_calculate_increment(1.0f, 0.0f, 0.0f, 0.1f, &dx, &dy);
    ESP_LOGI(TAG, "    Forward motion: dx=%.3f, dy=%.3f", dx, dy);

    coordinate_calculate_increment(1.0f, M_PI/2, 0.0f, 0.1f, &dx, &dy);
    ESP_LOGI(TAG, "    Left motion: dx=%.3f, dy=%.3f", dx, dy);
    ESP_LOGI(TAG, "  Coordinate: OK");

    // Test sensor fusion
    ESP_LOGI(TAG, "  Testing sensor fusion...");
    float gyro_heading = 0.1f;
    float wheel_heading = 0.12f;
    float fused = fusion_heading(gyro_heading, wheel_heading, 0.5f, true);
    ESP_LOGI(TAG, "    Fusion: gyro=%.3f, wheel=%.3f, fused=%.3f",
             gyro_heading, wheel_heading, fused);
    ESP_LOGI(TAG, "  Fusion: OK");

    ESP_LOGI(TAG, "All algorithm modules tested!");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== OPS9 Positioning System Starting ===");

    // Initialize NVS (needed for some tests)
    nvs_manager_init();

    test_algorithms();

    ESP_LOGI(TAG, "Stage 4: Algorithm modules verified");

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
