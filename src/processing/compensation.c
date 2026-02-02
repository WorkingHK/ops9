#include "compensation.h"
#include <math.h>

float compensation_temperature_drift(float omega_z, float temperature, const calibration_t *cal)
{
    if (!cal->valid) {
        return omega_z;
    }

    // Polynomial compensation: drift = c0 + c1*T + c2*T²
    float T = temperature;
    float drift = cal->temp_drift_coeffs[0] +
                  cal->temp_drift_coeffs[1] * T +
                  cal->temp_drift_coeffs[2] * T * T;

    return omega_z - drift;
}

float compensation_threshold(float omega_z, float threshold)
{
    // Dead zone to suppress drift at rest
    if (fabsf(omega_z) < threshold) {
        return 0.0f;
    }
    return omega_z;
}

float compensation_dynamic_state(float omega_z, float velocity, bool is_moving)
{
    // Apply different compensation based on motion state
    if (!is_moving) {
        // Stationary: aggressive drift suppression
        return compensation_threshold(omega_z, 0.01f);  // 0.01 rad/s threshold
    } else {
        // Moving: lighter threshold
        return compensation_threshold(omega_z, 0.001f);  // 0.001 rad/s threshold
    }
}
