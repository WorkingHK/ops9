#include "iakf.h"
#include <math.h>

void iakf_init(iakf_state_t *state, float Q, float R)
{
    state->x = 0.0f;
    state->P = 1.0f;
    state->Q = Q;
    state->R = R;
    state->K = 0.0f;
    state->innovation = 0.0f;
}

float iakf_update(iakf_state_t *state, float measurement)
{
    // Prediction step
    // x_pred = x (no state transition)
    // P_pred = P + Q
    float P_pred = state->P + state->Q;

    // Innovation
    state->innovation = measurement - state->x;

    // Adaptive R based on innovation
    float innovation_variance = fabsf(state->innovation);
    float R_adaptive = state->R * (1.0f + innovation_variance);

    // Kalman gain
    state->K = P_pred / (P_pred + R_adaptive);

    // Update step
    state->x = state->x + state->K * state->innovation;
    state->P = (1.0f - state->K) * P_pred;

    return state->x;
}
