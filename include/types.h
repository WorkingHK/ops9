#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// IMU state (updated by IMU task, read by processing task)
typedef struct {
    float omega_x;          // rad/s
    float omega_y;          // rad/s
    float omega_z;          // rad/s
    float accel_x;          // m/s²
    float accel_y;          // m/s²
    float accel_z;          // m/s²
    float temperature;      // °C
    uint64_t timestamp_us;  // microseconds
    bool valid;             // data validity flag
    SemaphoreHandle_t mutex;
} imu_state_t;

// Encoder state (updated by encoder task, read by processing task)
typedef struct {
    float velocity;         // m/s
    uint32_t pulse_count;   // total pulses
    uint64_t timestamp_us;  // microseconds
    bool valid;
    SemaphoreHandle_t mutex;
} encoder_state_t;

// Position state (updated by processing task, read by output tasks)
typedef struct {
    float x;                // meters
    float y;                // meters
    float heading;          // radians
    float confidence;       // 0.0-1.0
    uint64_t timestamp_us;  // microseconds
    bool fail_safe_mode;    // true if degraded
    SemaphoreHandle_t mutex;
} position_state_t;

// Calibration data (loaded from NVS at startup)
typedef struct {
    float gyro_bias[3];           // rad/s offset [x, y, z]
    float temp_drift_coeffs[3];   // polynomial coefficients
    float wheel_angle;            // non-orthogonal correction (radians)
    float iakf_params[5];         // IAKF tuning parameters
    bool valid;                   // calibration present
} calibration_t;

// Quaternion for attitude representation
typedef struct {
    float q0, q1, q2, q3;
} quaternion_t;

#endif // TYPES_H
