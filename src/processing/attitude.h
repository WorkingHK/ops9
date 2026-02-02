#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "types.h"

// Initialize quaternion to identity
void quaternion_init(quaternion_t *q);

// Normalize quaternion
void quaternion_normalize(quaternion_t *q);

// Update quaternion using 2nd-order Runge-Kutta
void quaternion_update_rk2(quaternion_t *q, float omega_x, float omega_y, float omega_z, float dt);

// Extract heading (yaw) from quaternion
float quaternion_to_heading(const quaternion_t *q);

#endif // ATTITUDE_H
