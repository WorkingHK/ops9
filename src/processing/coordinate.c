#include "coordinate.h"
#include <math.h>

void coordinate_calculate_increment(float velocity, float heading, float wheel_angle,
                                     float dt, float *dx, float *dy)
{
    // Apply non-orthogonal wheel correction using auxiliary coordinate system
    // Corrected heading = heading + wheel_angle
    float corrected_heading = heading + wheel_angle;

    // Calculate displacement in auxiliary frame
    float ds = velocity * dt;

    // Transform to global frame
    *dx = ds * cosf(corrected_heading);
    *dy = ds * sinf(corrected_heading);
}
