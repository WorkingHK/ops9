#ifndef COMPENSATION_H
#define COMPENSATION_H

#include "types.h"

// Apply temperature drift compensation
float compensation_temperature_drift(float omega_z, float temperature, const calibration_t *cal);

// Apply threshold processing (dead zone)
float compensation_threshold(float omega_z, float threshold);

// Apply dynamic state compensation
float compensation_dynamic_state(float omega_z, float velocity, bool is_moving);

#endif // COMPENSATION_H
