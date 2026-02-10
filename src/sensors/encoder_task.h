#ifndef ENCODER_TASK_H
#define ENCODER_TASK_H

#include "types.h"
#include "esp_err.h"

// Start encoder task with shared state and wheel diameters
esp_err_t encoder_task_start(encoder_state_t *shared_state,
                              float wheel_x_diameter,
                              float wheel_y_diameter);

#endif // ENCODER_TASK_H
