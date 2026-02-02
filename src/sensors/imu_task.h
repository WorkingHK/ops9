#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "types.h"
#include "esp_err.h"

// Initialize and start IMU task
esp_err_t imu_task_start(imu_state_t *shared_state);

#endif // IMU_TASK_H
