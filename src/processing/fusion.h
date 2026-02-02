#ifndef FUSION_H
#define FUSION_H

#include <stdbool.h>

// Fuse gyro heading with wheel odometry heading
float fusion_heading(float gyro_heading, float wheel_heading, float velocity, bool is_moving);

#endif // FUSION_H
