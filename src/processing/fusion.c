#include "fusion.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

float fusion_heading(float gyro_heading, float wheel_heading, float velocity, bool is_moving)
{
    if (!is_moving) {
        // Stationary: trust gyro more
        return gyro_heading;
    }

    // Moving: weighted fusion based on velocity
    // Higher velocity = trust wheel odometry more
    float velocity_weight = fminf(velocity / 1.0f, 1.0f);  // Normalize to [0, 1]
    float gyro_weight = 1.0f - velocity_weight;

    // Handle angle wrapping
    float diff = wheel_heading - gyro_heading;
    while (diff > M_PI) diff -= 2.0f * M_PI;
    while (diff < -M_PI) diff += 2.0f * M_PI;

    return gyro_heading + gyro_weight * diff;
}
