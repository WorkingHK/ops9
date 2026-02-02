#ifndef IAKF_H
#define IAKF_H

#include <stdint.h>

// IAKF state structure
typedef struct {
    float x;           // State estimate
    float P;           // Error covariance
    float Q;           // Process noise
    float R;           // Measurement noise
    float K;           // Kalman gain
    float innovation;  // Innovation (measurement - prediction)
} iakf_state_t;

// Initialize IAKF
void iakf_init(iakf_state_t *state, float Q, float R);

// Update IAKF with new measurement
float iakf_update(iakf_state_t *state, float measurement);

#endif // IAKF_H
