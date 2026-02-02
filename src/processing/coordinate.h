#ifndef COORDINATE_H
#define COORDINATE_H

// Calculate position increment with non-orthogonal wheel correction
void coordinate_calculate_increment(float velocity, float heading, float wheel_angle,
                                     float dt, float *dx, float *dy);

#endif // COORDINATE_H
