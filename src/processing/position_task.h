#ifndef POSITION_TASK_H
#define POSITION_TASK_H

#include "types.h"
#include "esp_err.h"

// Initialize and start position processing task
esp_err_t position_task_start(imu_state_t *imu_state,
                               encoder_state_t *encoder_state,
                               position_state_t *position_state,
                               calibration_t *calibration);

#endif // POSITION_TASK_H
