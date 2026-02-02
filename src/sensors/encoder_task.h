#ifndef ENCODER_TASK_H
#define ENCODER_TASK_H

#include "types.h"
#include "esp_err.h"

// Initialize and start encoder task
esp_err_t encoder_task_start(encoder_state_t *shared_state, float wheel_diameter);

#endif // ENCODER_TASK_H
